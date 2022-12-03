#include "CompilationOpts.h"

#ifdef USE_LED

#include <HardwareSerial.h>
#include <FS.h>
#include <SPIFFS.h>
#include <FastLED.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LedService.h"
#include "LedMeteorEffect.h"

void InterpolatedFunction::setParams(float xSize, int count)
{
    this->xEnd = xSize;
    this->deltaX = xSize / (float)(count - 1);
    this->cnt = count;
    delete[] values;
    values = new float[cnt];
    memset(values, 0, sizeof(values[0]) * cnt);
}

void InterpolatedFunction::setValue(int i, float y)
{
    if (i >= 0 && i < cnt) {
        values[i] = y;
    }
}

float InterpolatedFunction::getAt(float x) {
	float fSlot = x / deltaX;
    int slot = (int)floor(fSlot);
	float fract = fSlot - (float)slot;
    float v =
		(slot < 0 || slot >= cnt - 1) ? // shouldn't happen
        values[cnt - 1]
		: values[slot] + (values[slot + 1] - values[slot]) * fract;
	return v;
}

void Shape::paramsCalc()
{
    if (trailingPct > 100 || trailingPct < 0) {
        trailingPct = 0;
    }
    if (leadingPct > 100 || leadingPct < 0) {
        leadingPct = 0;
    }
    if (trailingPct + leadingPct > 100) {
        trailingPct = trailingPct * 100 / (trailingPct + leadingPct);
        leadingPct = 100 - trailingPct;
    }

    xTrailing = (float)lengthLeds * trailingPct / 100.0;
    xLeading = (float)lengthLeds * (100 - leadingPct) / 100.0;
    trailing.setParams(xTrailing, 20 + 1);
    leading.setParams((float)lengthLeds - xLeading, 20 + 1);

    for (int i = 0; i < 20; i++) {
        float x = (float)i / 20.0;
        float v = pow(3 * x * x - 2 * x * x * x, leadingGamma);
        float val = valMin + v * (valMax - valMin);
        trailing.setValue(i, val);
    }
    trailing.setValue(20, valMax);

    for (int i = 0; i < 20; i++) {
        float x = (float)i / 20.0;
        float v = pow(3 * x * x - 2 * x * x * x, trailingGamma);
        float val = valMax - v * (valMax - valMin);
        leading.setValue(i, val);
    }
    leading.setValue(20, valMin);
}

float Shape::shapeAt(float x)
{
    if (x >= 0 && x < xTrailing) {
        return trailing.getAt(x);
    } else if (x >= xLeading && x <= lengthLeds) {
        return leading.getAt(x - xLeading);
    } else if (x >= xTrailing && x < xLeading) {
		return valMax;
	} else {
        return valMin;
    }
}

LedMeteorEffect::LedMeteorEffect(int controllerId, LedMap1d *leds, int totalLedCount, int defaultLedStart, int defaultLedCount, const char *description)
: Effect(controllerId, leds, totalLedCount, description)
{
    this->ledStart = defaultLedStart;
    this->ledCount = defaultLedCount;
    isMappingEnabled = true;
}

LedMeteorEffect::~LedMeteorEffect()
{
}

void LedMeteorEffect::setCmd(ServiceCommands *cmd, const char *command, const char *help, uint8_t *data, int min, int max)
{
  cmd->registerIntData(
    ServiceCommands::IntDataBuilder(command, true)
    .cmd(command, true)
    .help(help)
    .vMin(min)
    .vMax(max)
    .setFn([this, data, command](int val, bool isLoading, String *msg) {
      *data = val;
      if (!isLoading) {
        calcFrameProperties();
        *msg = "Set "; msg->concat(command);
        msg->concat(" to "); msg->concat(*data);
      }
      return true;
    })
    .getFn([this, data]() {
      return *data;
    })
  );
};

void LedMeteorEffect::setCmd(ServiceCommands *cmd, const char *command, const char *help, int *data, int min, int max)
{
  cmd->registerIntData(
    ServiceCommands::IntDataBuilder(command, true)
    .cmd(command, true)
    .help(help)
    .vMin(min)
    .vMax(max)
    .setFn([this, data, command](int val, bool isLoading, String *msg) {
      *data = val;
      if (!isLoading) {
        calcFrameProperties();
        *msg = "Set "; msg->concat(command);
        msg->concat(" to "); msg->concat(*data);
      }
      return true;
    })
    .getFn([this, data]() {
      return *data;
    })
  );
};

void LedMeteorEffect::setCmd(ServiceCommands *cmd, const char *command, const char *help, float *data, float min, float max)
{
  cmd->registerFloatData(
    ServiceCommands::FloatDataBuilder(command, true)
    .cmd(command, true)
    .help(help)
    .vMin(min)
    .vMax(max)
    .setFn([this, data, command](float val, bool isLoading, String *msg) {
      *data = val;
      if (!isLoading) {
        calcFrameProperties();
        *msg = "Set "; msg->concat(command);
        msg->concat(" to "); msg->concat(*data);
      }
      return true;
    })
    .getFn([this, data]() {
      return *data;
    })
  );
};

void LedMeteorEffect::setCmd(ServiceCommands *cmd, const char *command, const char *help, bool *data)
{
  cmd->registerBoolData(
    ServiceCommands::BoolDataBuilder(command, true)
    .cmd(command, true)
    .help(help)
    .setFn([this, data, command](bool val, bool isLoading, String *msg) {
      *data = val;
      if (!isLoading) {
        calcFrameProperties();
        *msg = "Set "; msg->concat(command);
        msg->concat(" to "); msg->concat(*data);
      }
      return true;
    })
    .getFn([this, data]() {
      return *data;
    })
  );
};

void LedMeteorEffect::init(int millisPerFrame, CommandMgr *commandMgr, uint32_t frame)
{
    String controllerName = "led"; controllerName += controllerId;
    cmd = commandMgr->getServiceCommands(controllerName.c_str());

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("isEnabled", true)
            .cmdOn("enable")
            .cmdOff("disable")
            .helpOn("--> Enable this controller")
            .helpOff("--> Disable this controller")
            .ptr(&isEnabled)
            .setFn([this](bool val, bool isLoading, String *msg) {
                if (isLoading) {
                    isEnabled = val;
                } else {
                    if (isEnabled != val) {
                        isEnabled = val;
                        *msg = "Controller "; msg->concat(cmd->getServiceName());
                        msg->concat(isEnabled ? " enabled" : " disabled");

                        if (isEnabled) {
                            int waitFrames = timing.getNextDelay(rnd) / this->millisPerFrame;
                            startingFrame = lastFrame + waitFrames; // start as if we just finished a run
                            isRunning = false;
                        }

                    } else {
                        *msg = "Controller "; msg->concat(cmd->getServiceName());
                        msg->concat(" is already");
                        msg->concat(isEnabled ? " enabled" : " disabled");
                    }
                }
                return true;
            })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("ledStart", true)
            .cmd("ledStart")
            .help("--> Starting led, in hardware order; sets the ledCount to 1")
            .vMin(0)
            .vMax(totalLedCount - 2)
            .setFn([this](int val, bool isLoading, String *msg) {
                ledStart = val;
                if (ledStart + ledCount > this->totalLedCount) {
                    ledCount = this->totalLedCount - ledStart;
                    *msg = "Set ledStart to "; msg->concat(ledStart);
                    msg->concat(", set led count to"); msg->concat(ledCount);
                } else {
                    *msg = "Set ledStart to "; msg->concat(ledStart);
                    msg->concat(", led count remains "); msg->concat(ledCount);
                }
                return true;
            })
            .getFn([this]() {
                return ledStart;
            })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("ledCount", true)
            .cmd("ledCount")
            .help("--> Number of leds, consecutive to ledStart, in hardware order")
            .vMin(0)
            .setFn([this](int val, bool isLoading, String *msg) {
                if (ledStart + val > this->totalLedCount) {
                    *msg = "Led count "; msg->concat(val); msg->concat(" out of range, given ledStart = ");
                    msg->concat(ledStart);
                    msg->concat(" for totalLedCount = "); msg->concat(totalLedCount);
                    msg->concat(", maximum is "); msg->concat(totalLedCount - ledStart);
                } else {
                    ledCount = val;
                    *msg = "Set ledCount to "; msg->concat(ledCount);
                }
                return true;
            })
            .getFn([this]() {
                return ledCount;
            })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("mapping", true)
            .cmd("mapping")
            .help("mapping on|off --> Enable or disable mapping given in hardware config")
            .ptr(&isMappingEnabled)
    );

    setCmd(cmd, "valMin", "--> Minimum brightness, 0..255", &shape.valMin, 0, 255);
    setCmd(cmd, "valMax", "--> Maximum brightness, 0..255", &shape.valMax, 0, 255);

    setCmd(cmd, "lengthLeds", "--> Length in number of leds", &shape.lengthLeds, 0.1, 999999.0);
    setCmd(cmd, "trailingPct", "--> Percentage of length to use for the trailing", &shape.trailingPct, 0, 100);
    setCmd(cmd, "trailingGamma", "--> Gamma for the trailing", &shape.trailingGamma, 0.01, 1000.0);
    setCmd(cmd, "leadingPct", "--> Percentage of length to use for the leading", &shape.leadingPct, 0, 100);
    setCmd(cmd, "leadingGamma", "--> Gamma for the leading", &shape.leadingGamma, 0.01, 1000.0);

    setCmd(cmd, "saturation", "--> Color saturation (0..255)", &color.saturation, 0, 255);
    setCmd(cmd, "hueStart", "--> Starting hue (0..255)", &color.hueStart, 0, 255);
    setCmd(cmd, "hueStartRandom", "--> If true, starting hue will be random", &color.hueStartRandom);
    setCmd(cmd, "hueStartSeqChange", "--> Change in starting hue on each run (0..255)", &color.hueStartSeqChange, 0, 255);
    setCmd(cmd, "whiteAsRandomHuePct", "--> If true, and if hueStartRandom is selected, consider white as a possible random choice",
        &color.whiteAsRandomHuePct, 0, 100);
    setCmd(cmd, "hueShiftInShape", "--> Change in hue along the shape (float)", &color.hueShiftInShape, -999999.0, 999999.0);
    setCmd(cmd, "hueShiftInRun", "--> Change in hue during run (float)", &color.hueShiftInRun, -999999.0, 999999.0);

    setCmd(cmd, "pauseIsExponential", "--> If true, pause between runs has an exponential distribution, else uniform", &timing.isExponential);
    setCmd(cmd, "pauseDurationAvgMillis", "--> Average pause between runs, in millis", &timing.pauseDurationAvgMillis, 0, 999999);
    setCmd(cmd, "pauseDurationMaxMillis", "--> Maximum pause between runs, in millis", &timing.pauseDurationMaxMillis, 0, 999999);
    setCmd(cmd, "speedLedsPerSec", "--> Moving speed, in leds per second", &timing.speedLedsPerSec, -999999.0, 999999.0);

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("trigger", true)
            .cmdOn("trigger")
            .helpOn("--> Triggers a run, if not already running")
            .isPersistent(false)
            .setFn([this](int val, bool isLoading, String *msg) {
                if (!isEnabled) {
                    *msg = "Controller "; msg->concat(cmd->getServiceName());
                    *msg = "is not enabled";
                }
                if (isRunning) {
                    *msg = "Already running";
                } else {
                    startingFrame = lastFrame + 1;
                    *msg = "Triggered!";
                }
                return true;
            })
    );


    isEnabled = false;

    String msg;
    bool rc = cmd->load(nullptr, &msg);
    if (!rc) {
        Serial.printf("LedController %s: error loading defaults: %s\n", cmd->getServiceName(), msg.c_str());
    } else {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    timing.paramsCalc(); // so that we can call timing.getNextDelay()

    this->millisPerFrame = millisPerFrame;
    lastFrame = frame;
    runCount = 0;
    int waitFrames = timing.getNextDelay(rnd) / this->millisPerFrame;
    startingFrame = frame + waitFrames; // start as if we just finished a run
    isRunning = false;

    calcFrameProperties();
}

void LedMeteorEffect::onAfterLoad()
{
    calcFrameProperties();
}

void LedMeteorEffect::getOneLineStatus(String *msg)
{
    *msg += "Type: meteor, leds start at "; *msg += this->ledStart; *msg += ", for "; *msg += this->ledCount; *msg += " leds. Controller is ";
    *msg += (this->isEnabled ? "enabled." : "disabled.");
}


void LedMeteorEffect::setFrameDuration(int millis)
{
    this->millisPerFrame = millis;
    calcFrameProperties();
}

void LedMeteorEffect::calcFrameProperties()
{
    this->shape.paramsCalc();
    this->timing.paramsCalc();

    float durationInFrames = (int)((ledCount + shape.lengthLeds) * 1000 / (timing.speedLedsPerSec * millisPerFrame));
    hueShiftInRunPerFrame = color.hueShiftInRun / durationInFrames;

    hueShiftInShapePerLed = color.hueShiftInShape / shape.lengthLeds;
}

void LedMeteorEffect::calc(uint32_t frame)
{
    lastFrame = frame;

    if (!isEnabled) {
        return;
    }

    // do we need to start running?
    if (!isRunning && frame >= startingFrame) {
        // for the first time, set conditions for this run
        if (timing.speedLedsPerSec > 0) {
            curXStart = (float)(-shape.lengthLeds);
        } else if (timing.speedLedsPerSec < 0) {
            curXStart = (float)(ledCount - 1);
        } else {
            curXStart = (ledCount - shape.lengthLeds) / 2.0;
        }

        bool hueCalculated = false;
        if (color.whiteAsRandomHuePct != 0) {
            if (rnd.bernoulli(color.whiteAsRandomHuePct, 100)) {
                startingHue = 0;
                startingSaturation = 0;
                hueCalculated = true;
            }
        }
        if (!hueCalculated) {
            if (color.hueStartRandom) {
                startingHue = rnd.nextInt(0, 255);
            } else {
                startingHue = color.hueStart + runCount * color.hueStartSeqChange;
            }
            startingSaturation = color.saturation;
        }
        ++runCount;
        isRunning = true;
    }

    if (isRunning) {
        float ledsPerFrame = timing.speedLedsPerSec / 1000.0 * this->millisPerFrame;
        curXStart += ledsPerFrame;
        float fFirstLed = floor(curXStart);
        int firstLed = (int)fFirstLed;
        int lastLed = (int)ceil(curXStart + shape.lengthLeds);
        float xFraction = curXStart - fFirstLed; // amount to add at each integer led position in order obtain it's X-axis coordinate

        if (firstLed >= ledCount || lastLed < 0) {
            // we're finished with this run
            // set startingFrame in the future
            int waitFrames = timing.getNextDelay(rnd) / this->millisPerFrame;
            if (waitFrames == 0) {
                waitFrames = 1;
            }
            startingFrame = frame + waitFrames;
            isRunning = false;
        } else {
            // we're still running
            // render the leds

            // X coordinate of the first led that will be displayed; the X coordinate of each other led N
            // is found by adding N to xFirstLed
            float xFirstLed = 1 - xFraction;

            float firstLedWeight = 1.0 - xFraction;
            if (firstLed < 0) {
                xFirstLed += -firstLed;
                firstLed = 0;
                firstLedWeight = 1.0;
            }
            float lastLedWeight = 1.0 - (lastLed - (curXStart + shape.lengthLeds));
            if (lastLed > ledCount - 1) {
                lastLed = ledCount - 1;
                lastLedWeight = 1.0;
            }

            for (int i = firstLed; i < lastLed; i++) {
                float x = xFirstLed + (i - firstLed);
                float val = shape.shapeAt(timing.speedLedsPerSec > 0 ? x : shape.lengthLeds - x);
                // for first and last leds, weight val according to part of x within [curXStart, x] and [x, curXStart + shape.lengthLeds]
                if (i == firstLed) {
                    val *= firstLedWeight;
                }
                if (i == lastLed - 1) {
                    val *= lastLedWeight;
                }

                int saturation = startingSaturation;
                int hue = startingHue;
                if (color.hueShiftInShape != 0.0 || color.hueShiftInRun != 0.0) {
                    hue += (int)(hueShiftInShapePerLed * (timing.speedLedsPerSec > 0 ? x : shape.lengthLeds - x)
                         + hueShiftInRunPerFrame * (frame - startingFrame)
                         + 0.5);
                }

                if (isMappingEnabled) {
                    leds->at(ledStart + i)->setHSV(hue, saturation, (int)(val + 0.5));
                } else {
                    leds->atWithoutMapping(ledStart + i)->setHSV(hue, saturation, (int)(val + 0.5));
                }
            }
        }
    }
}

/*

stats

*/


void LedMeteorEffect::printStats()
{
    Serial.printf("  %s -- Stats TODO\n", cmd->getServiceName());
}

void LedMeteorEffect::printStats(String *msg)
{
    *msg += cmd->getServiceName(); *msg += " -- Stats TODO";
}



#endif // USE_LED
