#include <CompilationOpts.h>

#ifdef USE_SERVO

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "ServoService.h"
#include <ESP32Servo.h>
#include <TFT_eSPI.h>

void ServoService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("Servo");
    this->cmd = commandMgr->getServiceCommands("servo");

    servoPin = SERVO_PIN;
    autoIdleMillis = 0;
    minPulseWidth = MIN_PULSE_WIDTH;
    maxPulseWidth = MAX_PULSE_WIDTH;
    maxPosition = 1000;
    position = 0;
    lcdBacklightLevel = 100;
    slack = 0;
    slackPositioningDurationMillis = 10000;
    isPrePositioning = false;

    lcdAnimate = false;
    lcdLoopCnt = 0;
    lcdFps = 10;
    lcdIsDma = false;
    lcdBgColor = 0x000004;
    isEnabled = true;

    initCommands();

    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    eventLoop->registerTimer(&positioningTimer);
    eventLoop->registerTimer(&drivingTimer);
    drivingTimer.setCallback([this](UEventLoopTimer *timer) {
        logger->debug("Disabling because of autoIdle timeout of {} ms", autoIdleMillis);
        disable();
    });

//    ESP32PWM::allocateTimer(0); -- we're using ESP32PWM for lcd backlight too
    servo.setPeriodHertz(50);

    if (isEnabled) {
        // isEnabled from loading or default
        isEnabled = false; // so that we can execute enable()
        enable(); // will set isEnable
    }

    // LCD

    digitalWrite(TFT_BLK, LOW);
    pinMode(TFT_BLK, OUTPUT);
    lcdInit();
    setLcdBacklight(lcdBacklightLevel, true, nullptr);

    eventLoop->registerTimer(&lcdTimer);
    lcdLoopCnt = 0;
    lcdTimer.setCallback([this](UEventLoopTimer *timer) {
        if (lcdAnimate) {
            ++lcdLoopCnt;
            long tm1 = micros();
            lcdLoop();
            int duration = (int)(micros() - tm1);
            if (lcdLoopCnt % 256 == 0) {
                logger->trace("Loop duration: {} micros ({} fps max)", duration, (int)(1000000 / duration));
            }
            int fpsDuration = 1000000 / lcdFps;
            int toWait = (duration < fpsDuration ? fpsDuration - duration : 1000 /* if too late, wait just 1 ms */);
            lcdTimer.setTimeout(toWait / 1000);
        }
    });
    if (lcdAnimate) {
        lcdTimer.setTimeout(100);
    }
}

void ServoService::initCommands()
{
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("pin", true)
        .cmd("pin")
        .help("--> Set led pin, set to -1 for no led")
        .vMin(-1)
        .vMax(99)
        .ptr(&servoPin)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            servoPin = val;
            if (!isLoading && isEnabled) {
                disable();
                enable();
            }
            *msg = "Servo pin set to "; *msg += servoPin;
            return true;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("minPulseWidthMicros", true)
        .cmd("minPulseWidthMicros")
        .help("--> Set minimum pulse width (>= 500 us), corresponds to position 0")
        .vMin(500)
        .vMax(2500)
        .ptr(&minPulseWidth)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            minPulseWidth = val;
            if (!isLoading && isEnabled) {
                disable();
                enable();
            }
            *msg = "Minimum pulse width set to "; *msg += minPulseWidth;
            return true;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("maxPulseWidthMicros", true)
        .cmd("maxPulseWidthMicros")
        .help("--> Set maximum pulse width (>= 2500 us), corresponds to maxPosition")
        .vMin(500)
        .vMax(2500)
        .ptr(&maxPulseWidth)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            maxPulseWidth = val;
            if (!isLoading && isEnabled) {
                disable();
                enable();
            }
            *msg = "Maximum pulse width set to "; *msg += maxPulseWidth;
            return true;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("maxPosition", true)
        .cmd("maxPosition")
        .help("--> Set maximum position (minimum is 0) -- maxPulseWidthMicros will corresponnd to maximum position")
        .vMin(1)
        .ptr(&maxPosition)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            if (!isLoading) {
                position = position / maxPosition * val;
            }
            maxPosition = val;
            if (!isLoading && isEnabled) {
                disable();
                enable();
            }
            *msg = "Maximum pulse width set to "; *msg += maxPulseWidth;
            return true;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("slack", true)
        .cmd("slack")
        .help("--> When positioning counter-clockwise, first go to position - slack, then go clockwise to final position")
        .vMin(1)
        .ptr(&slack)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            if (val > maxPosition) {
                *msg = "Slack must be less than maxPosition (";
                msg->concat(maxPosition);
                msg->concat(")");
                return true;
            }
            slack = val;
            *msg = "Set slack to "; *msg += slack;
            return true;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("slackPositioningDurationMillis", true)
        .cmd("slackPositioningDurationMillis")
        .help("--> Milliseconds to wait for positioning to position - slack, before going clockwise to final position")
        .vMin(0)
        .vMax(60000)
        .ptr(&slackPositioningDurationMillis)
    );
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("isPrePositioning", true)
        .cmd("isPrePositioning")
        .help("--> True if we're pre-positioning to position - slack, before going to final position")
        .isPersistent(false)
        .getFn([this]() -> bool {
            return isPrePositioning;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("position", true)
        .cmd("position")
        .help("--> Set servo position, 0 - maxPosition")
        .vMin(0)
        .ptr(&position)
        .isPersistent(false)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            if (isLoading) {
                // we don't load position (it's not persistent, but just in case)
                return true;
            }
            if (val > maxPosition) {
                *msg = "Position must be less than ";
                msg->concat(maxPosition);
                msg->concat(" (current maxPosition)");
                return true;
            }
            if (slack > 0 && val < slack) {
                *msg = "Position must be greater than ";
                msg->concat(slack);
                msg->concat(" (current slack value)");
                return true;
            }
            if (!isEnabled) {
                enable();
            }

            int pos;
            isPrePositioning = false;
            positioningTimer.cancelTimeout();
            drivingTimer.cancelTimeout();
            if (val < position && slack > 0) { // we're moving backwords -- go to position - slack, then go to position
                isPrePositioning = true;
                positioningTimer.setTimeout([this](UEventLoopTimer *timer) {
                    logger->trace("Positioning after slack of {} to position {}", slack, position);
                    int pulseWidth = minPulseWidth + (uint32_t)position * ((maxPulseWidth + 1) - minPulseWidth) / (maxPosition + 1);
                    servo.write(pulseWidth);
                    if (autoIdleMillis > 0) {
                        drivingTimer.setTimeout(autoIdleMillis);
                    }
                    isPrePositioning = false;
                }, slackPositioningDurationMillis);
                pos = val - slack;
                position = val;
            } else {
                pos = val;
                position = val;
                if (autoIdleMillis > 0) {
                    drivingTimer.setTimeout(autoIdleMillis);
                }
            }
            int pulseWidth = minPulseWidth + (uint32_t)pos * ((maxPulseWidth + 1) - minPulseWidth) / (maxPosition + 1);
            servo.write(pulseWidth);


            *msg = "Servo position set to "; *msg += position;
            if (pos != position) {
                msg->concat(" (with intermediary position "); msg->concat(position); msg->concat(")");
            }
            *msg += " (maximum: "; *msg += maxPosition;
            *msg += "; pulse width: "; *msg += pulseWidth; *msg += " micros)";
            return true;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("autoIdle", true)
        .cmd("autoIdle")
        .help("--> Timeout in milliseconds for stopping the servo data line after a move. Set to 0 to keep always active (default).")
        .vMin(0)
        .ptr(&autoIdleMillis)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            autoIdleMillis = val;
            if (!isLoading && isEnabled) {
                disable();
                enable();
            }
            return true;
        })
    );

    // LCD
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("lcdBacklightLevel", true)
        .cmd("lcdBacklightLevel")
        .help("--> Set backlight level: 0 for off, 100 for on, or in between.")
        .ptr(&lcdBacklightLevel)
        .vMin(0)
        .vMax(100)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            if (isLoading) {
                lcdBacklightLevel = val;
                return true;
            }
            setLcdBacklight(val, false, msg);
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("lcdAnimate", true)
        .cmd("lcdAnimate")
        .help("--> Enable or disable lcd animation.")
        .ptr(&lcdAnimate)
        .setFn([this](bool val, bool isLoading, String *msg) -> bool {
            if (isLoading) {
                lcdAnimate = val;
                return true;
            }
            if (val && !lcdAnimate) {
                lcdAnimate = true;
                lcdTimer.setTimeout(100);
                *msg = String("Started animation");
            } else if (!val && lcdAnimate) {
                lcdAnimate = false;
                lcdTimer.cancelTimeout();
                *msg = String("Stopped animation");
            } else {
                *msg = String("Animation is already ") + (lcdAnimate ? "On" : "Off");
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("lcdFps", true)
        .cmd("lcdFps")
        .help("--> LCD animation frames per second.")
        .vMin(1)
        .vMax(1000)
        .ptr(&lcdFps)
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("lcdIsDma", true)
        .cmd("lcdIsDma")
        .help("--> LCD in DMA mode.")
        .ptr(&lcdIsDma)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("lcdBgColor", true)
        .cmd("lcdBgColor")
        .isShowAsHex(true)
        .help("--> Background color.")
        .ptr(&lcdBgColor)
    );

}

void ServoService::enable()
{
    if (!isEnabled) {
        servo.attach(servoPin, minPulseWidth, maxPulseWidth);
        isEnabled = true;
        logger->debug("Servo enabled");
    }
}

void ServoService::disable()
{
    if (isEnabled) {
        drivingTimer.cancelTimeout();
        servo.detach();
        isEnabled = false;
        logger->debug("Servo disabled");
    }
}

void ServoService::setLcdBacklight(int val, bool isInit, String *msg)
{
// gamma 2.8
//    static const int levels[] = { 0,15,104,323,723,1351,2251,3466,5038,7006,9410,12288,15678,19617,24141,29285,35085,41576,48792,56767,65535 };
// gamma 2.2
    static const int levels[] = { 0,90,413,1009,1900,3104,4636,6508,8730,11312,14263,17590,21301,25403,29901,34802,40112,45835,51976,58542,65535 };

// levels[i] = (int)(pow((float)i / (float)max_in, gamma) * max_out + 0.5));
    if (val < 0) {
        val = 0;
    } else if (val > 100) {
        val = 100;
    }
    if (val == 0 || val == 100) {
        if (lcdBacklightPwm.attached()) {
            lcdBacklightPwm.detachPin(TFT_BLK);
        }
        lcdBacklightLevel = val;
        digitalWrite(TFT_BLK, lcdBacklightLevel == 100 ? HIGH : LOW);
        if (msg != nullptr) {
            *msg = String("Set backlight to ") + (lcdBacklightLevel ? "ON" : "OFF");
        }
    } else {
        if (!lcdBacklightPwm.attached()) {
            lcdBacklightPwm.attachPin(TFT_BLK, 400, 16);
        }
        lcdBacklightLevel = val;
        int x1 = val * 20 / 100;
        int xrem = val - x1 * 100 / 20;
        int fx1 = levels[x1];
        int fx2 = levels[x1 + 1];
        int res = fx1 + (fx2 - fx1) * xrem * 20 / 100;
        lcdBacklightPwm.write(res);
        if (msg != nullptr) {
            *msg = String("Backlight set to ") + lcdBacklightLevel;
        }
    }
}

/**************************************
 *
 * LCD
 *
 **************************************/

// Color depth has to be 16 bits if DMA is used to render image
#define COLOR_DEPTH 16

// 128x128 for a 16 bit colour Sprite (32Kbytes RAM)
// Maximum is 181x181 (64Kbytes) for DMA -  restricted by processor design
#define IWIDTH  128
#define IHEIGHT 128

// Size of cube image
// 358 is max for 128x128 sprite, too big and pixel trails are drawn...
#define CUBE_SIZE 358


void ServoService::lcdInit()
{
    // Create two sprites for a DMA toggle buffer
    spr[0] = new TFT_eSprite(&tft);
    spr[1] = new TFT_eSprite(&tft);
    // Toggle buffer selection
    sprSel = 0;
    // Define the cube face colors
    palette[0] = TFT_WHITE;
    palette[1] = TFT_GREENYELLOW;
    palette[2] = TFT_YELLOW;
    palette[3] = TFT_PINK;
    palette[4] = TFT_MAGENTA;
    palette[5] = TFT_CYAN;

    // size / 2 of cube edge
    d = 15;
    float tpx[] = { -d,  d,  d, -d, -d,  d,  d, -d };
    memcpy(px, tpx, sizeof(tpx));
    float tpy[] = { -d, -d,  d,  d, -d, -d,  d,  d };
    memcpy(py, tpy, sizeof(tpy));
    float tpz[] = { -d, -d, -d, -d,  d,  d,  d,  d };
    memcpy(pz, tpz, sizeof(tpz));

    // Define the triangles
    // The order of the vertices MUST be CCW or the
    // shoelace method won't work to detect visible edges
    int tfaces[12][3] = {
        {0, 1, 4},
        {1, 5, 4},
        {1, 2, 5},
        {2, 6, 5},
        {5, 7, 4},
        {6, 7, 5},
        {3, 4, 7},
        {4, 3, 0},
        {0, 3, 1},
        {1, 3, 2},
        {2, 3, 6},
        {6, 3, 7}
    };
    memcpy(faces, tfaces, sizeof(tfaces));

    // mapped coordinates on screen
    memset(p2x, 0, sizeof(p2x));
    memset(p2y, 0, sizeof(p2y));
    // rotation angle in radians
    memset(r, 0, sizeof(r));

    // Sprite draw position
    xpos = 0;
    ypos = 0;

    // 3 axis spin control
    spinX = true;
    spinY = true;
    spinZ = true;

    tft.init();
    tft.fillScreen(lcdBgColor);

    xpos = 0;
    ypos = (tft.height() - IHEIGHT) / 2;

    // Define cprite colour depth
    spr[0]->setColorDepth(COLOR_DEPTH);
    spr[1]->setColorDepth(COLOR_DEPTH);

    // Create the 2 sprites
    sprPtr[0] = (uint16_t*)spr[0]->createSprite(IWIDTH, IHEIGHT);
    sprPtr[1] = (uint16_t*)spr[1]->createSprite(IWIDTH, IHEIGHT);

    // Define text datum and text colour for Sprites
    spr[0]->setTextColor(TFT_BLACK);
    spr[0]->setTextDatum(MC_DATUM);
    spr[1]->setTextColor(TFT_BLACK);
    spr[1]->setTextDatum(MC_DATUM);

    // DMA - should work with ESP32, STM32F2xx/F4xx/F7xx processors
    // NOTE: >>>>>> DMA IS FOR SPI DISPLAYS ONLY <<<<<<
    tft.initDMA(); // Initialise the DMA engine (tested with STM32F446 and STM32F767)

    // Random movement direction
    dx = 1; if (random(2)) dx = -1;
    dy = 1; if (random(2)) dy = -1;

}

void ServoService::lcdLoop() {
    // Grab exclusive use of the SPI bus
    tft.startWrite();

    // Pull it back onto screen if it wanders off
    if (xpos < -xmin) {
      dx = 1;
      bounce = true;
    }
    if (xpos >= tft.width() - xmax) {
      dx = -1;
      bounce = true;
    }
    if (ypos < -ymin) {
      dy = 1;
      bounce = true;
    }
    if (ypos >= tft.height() - ymax) {
      dy = -1;
      bounce = true;
    }

    if (bounce) {
      // Randomise spin
      if (random(2)) spinX = true;
      else spinX = false;
      if (random(2)) spinY = true;
      else spinY = false;
      if (random(2)) spinZ = true;
      else spinZ = false;
      bounce = false;
    }

    xmin = IWIDTH / 2; xmax = IWIDTH / 2; ymin = IHEIGHT / 2; ymax = IHEIGHT / 2;
    drawCube();

    if (lcdIsDma) {
        if (tft.dmaBusy()) {
            // must slow down, TODO
            sleep(100);
        }
        tft.pushImageDMA(xpos, ypos, IWIDTH, IHEIGHT, sprPtr[sprSel]);
        sprSel = !sprSel;
    } else {
        spr[sprSel]->pushSprite(xpos, ypos); // Blocking write (no DMA) 115fps
    }

    // Change coord for next loop
    xpos += dx;
    ypos += dy;

    tft.endWrite();
}

/**
  Detected visible triangles. If calculated area > 0 the triangle
  is rendered facing towards the viewer, since the vertices are CCW.
  If the area is negative the triangle is CW and thus facing away from us.
*/
int ServoService::shoelace(int x1, int y1, int x2, int y2, int x3, int y3) {
  // (x1y2 - y1x2) + (x2y3 - y2x3)
  return x1 * y2 - y1 * x2 + x2 * y3 - y2 * x3 + x3 * y1 - y3 * x1;
}

/**
  Rotates and renders the cube.
**/
void ServoService::drawCube()
{
  double speed = 90;
  if (spinX) r[0] = r[0] + PI / speed; // Add a degree
  if (spinY) r[1] = r[1] + PI / speed; // Add a degree
  if (spinZ) r[2] = r[2] + PI / speed; // Add a degree

  if (r[0] >= 360.0 * PI / 90.0) r[0] = 0;
  if (r[1] >= 360.0 * PI / 90.0) r[1] = 0;
  if (r[2] >= 360.0 * PI / 90.0) r[2] = 0;

  float ax[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  float ay[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  float az[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // Calculate all vertices of the cube
  for (int i = 0; i < 8; i++)
  {
    float px2 = px[i];
    float py2 = cos(r[0]) * py[i] - sin(r[0]) * pz[i];
    float pz2 = sin(r[0]) * py[i] + cos(r[0]) * pz[i];

    float px3 = cos(r[1]) * px2 + sin(r[1]) * pz2;
    float py3 = py2;
    float pz3 = -sin(r[1]) * px2 + cos(r[1]) * pz2;

    ax[i] = cos(r[2]) * px3 - sin(r[2]) * py3;
    ay[i] = sin(r[2]) * px3 + cos(r[2]) * py3;
    az[i] = pz3 - 150;

    p2x[i] = IWIDTH / 2 + ax[i] * CUBE_SIZE / az[i];
    p2y[i] = IHEIGHT / 2 + ay[i] * CUBE_SIZE / az[i];
  }

  // Fill the buffer with color 0 (Black)
  spr[sprSel]->fillSprite(lcdBgColor);

  for (int i = 0; i < 12; i++) {

    if (shoelace(p2x[faces[i][0]], p2y[faces[i][0]], p2x[faces[i][1]], p2y[faces[i][1]], p2x[faces[i][2]], p2y[faces[i][2]]) > 0) {
      int x0 = p2x[faces[i][0]];
      int y0 = p2y[faces[i][0]];
      int x1 = p2x[faces[i][1]];
      int y1 = p2y[faces[i][1]];
      int x2 = p2x[faces[i][2]];
      int y2 = p2y[faces[i][2]];

      xmin = min(xmin, x0);
      ymin = min(ymin, y0);
      xmin = min(xmin, x1);
      ymin = min(ymin, y1);
      xmin = min(xmin, x2);
      ymin = min(ymin, y2);
      xmax = max(xmax, x0);
      ymax = max(ymax, y0);
      xmax = max(xmax, x1);
      ymax = max(ymax, y1);
      xmax = max(xmax, x2);
      ymax = max(ymax, y2);

      spr[sprSel]->fillTriangle(x0, y0, x1, y1, x2, y2, palette[i / 2]);
      if (i % 2) {
        int avX = 0;
        int avY = 0;
        for (int v = 0; v < 3; v++) {
          avX += p2x[faces[i][v]];
          avY += p2y[faces[i][v]];
        }
        avX = avX / 3;
        avY = avY / 3;
      }
    }
  }

}


#endif
