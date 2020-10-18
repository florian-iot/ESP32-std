#include "CompilationOpts.h"

#ifdef USE_LED

#include <HardwareSerial.h>
#include <FS.h>
#include <SPIFFS.h>
#include <FastLED.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LedService.h"
#include "LedSparkEffect.h"
#include "LedMeteorEffect.h"

Effect::Effect(int controllerId, CRGB *leds, int totalLedCount, const char *description)
{
    this->leds = leds;
    this->totalLedCount = totalLedCount;
    this->controllerId = controllerId;
    this->description = description;
}

Effect::~Effect()
{
}

bool LedService::loadHardwareAndControllers(UEventLoop *eventLoop, CommandMgr *commandMgr)
{
    // read configuration from file led-hardware.conf.json
    for (int i = 0; i < LED_HARDWARE_COUNT; i++) {
        hardware[i].isConfigured = false;
    }

    DynamicJsonBuffer buf;
    JsonObject *params;
    const char *configFile = "/led-hardware.conf.json";
    Serial.printf("Loading %s\n", configFile);
    if (!SPIFFS.exists(configFile)) {
        // empty params
        params = &buf.createObject();
        (*params)["hardware"] = buf.createArray();
        (*params)["controlles"] = buf.createArray();

        const char *fileExample = R"(
{
    "hardware": [
        // { chip: "WS2811", "rgbOrder": "RGB", "pin": 19, "count": 0 }
    ],
    "controllers": [
        // must be a non-empty array of
        // { id:<n, defaults to index in the array>, type: "spark" / "meteor", description, ledStart, ledCount }
    ]
}
        )";
        File f = SPIFFS.open(configFile, "w");
        if (f) {
            f.println(fileExample);
            f.close();
        }
    } else {
        File f = SPIFFS.open(configFile, "r");
        JsonObject &cfg = buf.parseObject(f);
        f.close();
        if (!cfg.success()) {
            initializationError = String("Error reading ") + configFile + ", incorrect JSON";
            Serial.println(initializationError.c_str());
            params = nullptr;
            return false;
        } else {
            Serial.printf("Loaded %s: ", configFile);
            cfg.prettyPrintTo(Serial);
            Serial.println();
            params = &cfg;
        }
    }

    const JsonVariant &h = (*params)["hardware"];
    if (h.size() >= LED_HARDWARE_COUNT) {
        initializationError = String("In config file ") + configFile + ", array \"hardware\" has "
            + h.size() + " elements, more than allowed " + LED_HARDWARE_COUNT;
        Serial.println(initializationError);
        return false;
    }
    totalLedCount = 0;
    for (int i = 0; i < h.size(); i++) {
        const char *chip = h[i]["chip"];
        int pin = h[i]["pin"];
        int count = h[i]["count"];
        if (pin < 0 || pin > 40 || count <= 0 || count > 10000) {
            initializationError = String("In config file ") + configFile
                + ", bad values chip " + (chip == nullptr ? "NULL" : chip)
                + ", pin " + pin + ", count " + count;
            Serial.println(initializationError);
            return false;
        } else {
            hardware[i].ledChip = chip;
            hardware[i].rgbOrder = h[i]["rgbOrder"] | "RGB";
            hardware[i].ledPin = pin;
            hardware[i].count = count;
            hardware[i].isConfigured = true;
            totalLedCount += count;
        }
    }

    // required here, because we create controllers just below and "leds" is needed
    leds = new CRGB[totalLedCount];

    const JsonVariant &ctrl = (*params)["controllers"];
    if (!ctrl) {
        initializationError = String("Expecting \"controllers\" config file ") + configFile;
        Serial.println(initializationError);
        return false;
    }
    if (ctrl.size() <= 0 || ctrl.size() > 100) {
        initializationError = String("The element \"controllers\" in config file ") + configFile
            + " must be a non-empty array of { \"type\": <\"spark\"|\"meteor\"> }";
        Serial.println(initializationError);
        return false;
    }
    controllerCount = ctrl.size();
    controllers = new Effect*[controllerCount];
    for (int i = 0; i < controllerCount; i++) {
        const char *type = ctrl[i]["type"];
        if (type == nullptr || type[0] == '0') {
            initializationError = String("Expecting \"type\" in controllers[") + i + "] in config file " + configFile;
            Serial.println(initializationError);
            return false;
        }
        int id = ctrl[i]["id"] | i;
        int ledStart = ctrl[i]["ledStart"] | 0;
        int ledCount = ctrl[i]["ledCount"] | totalLedCount;
        if (strcmp(type, "spark") == 0) {
            controllers[i] = new EffectSparks(id, leds, totalLedCount, ledStart, ledCount, ctrl[i]["description"] | "" );
        } else if (strcmp(type, "meteor") == 0) {
            // TODO EffectMeteor
            controllers[i] = new LedMeteorEffect(id, leds, totalLedCount, ledStart, ledCount, ctrl[i]["description"] | "" );
        } else {
            initializationError = String("Unrecognized controller type \"") + type + "\" in config file " + configFile;
            Serial.println(initializationError);
            return false;
        }
    }

    return true;
}

void LedService::init(UEventLoop *eventLoop, CommandMgr *commandMgr)
{
    this->eventLoop = eventLoop;
    isInitialized = false;

    isEnabled = false;
    leds = nullptr;
    controllers = nullptr;
    controllerCount = 0;
    totalLedCount = 0;

    int rc = loadHardwareAndControllers(eventLoop, commandMgr);
    if (!rc) {
        if (initializationError == nullptr) {
            initializationError = "Unspecified error while loading hardware configuration";
        }
        Serial.printf("LED service hardware configuration was not performed, led service will not be active; initialization error: %s\n",
            initializationError.c_str());
        isEnabled = false;
    }

    cmd = commandMgr->getServiceCommands("led");

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("isEnabled", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .helpOn("--> Enable the service")
        .helpOff("--> Disable the service")
        .ptr(&isEnabled)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (!isLoading) {
                if (val && !isEnabled) {
                    if (initializationError != nullptr) {
                        *msg = "Led service cannot be enabled because it is not initialized, error: ";
                        *msg += initializationError;
                    } else {
                        enable();
                        *msg = "Led service enabled";
                    }
                } else if (!val && isEnabled) {
                    disable();
                    *msg = "Led service disabled";
                } else {
                    *msg = "Led service is already "; *msg += (isEnabled ? "enabled" : "disabled");
                }
            } else {
                isEnabled = val;
            }
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("initializationError", true)
            .cmd("initializationError")
            .help("--> Initialization error message, if there was an initialization failure")
            .isPersistent(false)
            .getFn([this](String *str) { *str = initializationError; })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("brightness", true)
            .cmd("brightness")
            .help("--> Global brightness, a number between 0 and 255")
            .vMin(0)
            .vMax(255)
            .setFn([this](int val, bool isLoading, String *msg) {
                brightness = val;
                if (!isLoading) {
                    FastLED.setBrightness(brightness);
                    *msg = "Global brightness set to "; *msg += val;
                }
                return true;
            })
            .getFn([this]() {
                return FastLED.getBrightness();
            })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("powerLimit", true)
            .cmd("powerLimit")
            .help("--> Power limit, in milliamps at 5V")
            .vMin(1)
            .vMax(999999)
            .ptr(&powerLimit)
            .setFn([this](int val, bool isLoading, String *msg) {
                powerLimit = val;
                if (!isLoading) {
                    FastLED.setMaxPowerInVoltsAndMilliamps(5, powerLimit);
                    *msg = "Power limit set to ";
                    *msg += powerLimit;
                    *msg += " milliamps at 5V";
                }
                return true;
            }));

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("freq", true)
            .cmd("freq")
            .help("--> Frequency of led updates, in frames/second")
            .vMin(1)
            .vMax(500)
            .setFn([this](int val, bool isLoading, String *msg) {
                intervalMillis = 1000 / val;
                if (!isLoading) {
                    for (int i = 0; i < controllerCount; i++) {
                        controllers[i]->setFrameDuration(intervalMillis);
                    }
                    if (isEnabled) {
                        timer.setInterval(intervalMillis);
                    }
                    *msg = "Frequency set to ";
                    *msg += 1000 / intervalMillis;
                    *msg += " frames/sec, frame interval: ";
                    *msg += intervalMillis;
                    *msg += " millis";
                }
                return true;
            })
            .getFn([this]() {
                return 1000 / intervalMillis;
            }));

    cmd->onBeforeLoad([this](String *msg) {
        if (isEnabled) {
            disable();
        }
        return true;
    });
    cmd->onAfterLoad([this](String *msg) {
        if (isEnabled && initializationError == nullptr) {
            for (int i = 0; i < controllerCount; i++) {
                controllers[i]->setFrameDuration(intervalMillis);
            }
            enable();
        }
    });

    cmd->onAfterStatus([this](String *msg) {
        *msg += "Total led count: "; *msg += totalLedCount; *msg += "\n";
        *msg += "Hardware:\n";
        for (int i = 0; i < sizeof(hardware) / sizeof(hardware[0]); i++) {
            *msg += "    "; *msg += i;
            if (hardware[i].isConfigured) {
                *msg += " ";
                *msg += hardware[i].ledChip;
                *msg += ", pin "; *msg += hardware[i].ledPin;
                *msg += ", count: "; *msg += hardware[i].count;
                *msg += "\n";
            } else {
                *msg += " not configured\n";
            }
        }

        *msg += "Controllers: "; *msg += controllerCount; *msg += "\n";
        for (int i = 0; i < controllerCount; i++) {
            *msg += "    led"; *msg += i; *msg += ": "; controllers[i]->getOneLineStatus(msg); *msg += "\n";
        }

        if (isEnabled) {
            float fMillis = avgFrameInterval / 1000.0f;
            *msg += "Target interval: "; *msg += intervalMillis; *msg += ", Avg frame interval: "; *msg += fMillis;
            *msg += " ms ("; *msg += (1000.0 / fMillis); *msg += " frames/sec)\n";

            *msg += "Avg duration micros: "; *msg += (avgTotalDuration >> 4); *msg += " total, "; *msg += ((avgTotalDuration / totalLedCount) >> 4);
            *msg += " per led, "; *msg += ((avgShowDuration / totalLedCount) >> 4); *msg += " for showing\n";
            for (int i = 0; i < controllerCount; i++) {
                controllers[i]->printStats(msg);
                *msg += "\n";
            }
        } else {
            *msg += "Led service is currently disabled";
            if (initializationError != nullptr) {
                *msg += " -- initialization error: ";
                *msg += initializationError;
            }
        }
    });

    // Defaults

    powerLimit = 1000;
    intervalMillis = 33;
    frame = 0;
    isEnabled = false;
    brightness = 128;
    avgShowDuration = 0;
    avgTotalDuration = 0;
    avgFrameInterval = 0;

    // Controllers

    if (initializationError == nullptr) {
        String controllerName;
        for (int c = 0; c < controllerCount; c++) {
            Effect *controller = controllers[c];
            Serial.printf("Initializing controller led%d\n", c);
            controller->init(intervalMillis, commandMgr, frame);
        }
    }

    String msg;
    rc = cmd->load(nullptr, &msg);
    if (!rc) {
        initializationError = msg;
        Serial.printf("LedService: error loading defaults: %s\n", msg.c_str());
    } else {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    // initialize hardware
    if (initializationError == nullptr) {
        int curPos = 0;
        for (int i = 0; i < LED_HARDWARE_COUNT; i++) {
            if (!hardware[i].isConfigured) {
                continue;
            }
            int ledPin = hardware[i].ledPin;

    #define ADD_LEDS_PER_PIN(CHIP, RGBORDER) \
                case LED_PIN_1: FastLED.addLeds<CHIP, LED_PIN_1, RGBORDER>(leds, curPos, hardware[i].count); break; \
                case LED_PIN_2: FastLED.addLeds<CHIP, LED_PIN_2, RGBORDER>(leds, curPos, hardware[i].count); break; \
                case LED_PIN_3: FastLED.addLeds<CHIP, LED_PIN_3, RGBORDER>(leds, curPos, hardware[i].count); break; \
                case LED_PIN_4: FastLED.addLeds<CHIP, LED_PIN_4, RGBORDER>(leds, curPos, hardware[i].count); break; \
                case LED_PIN_5: FastLED.addLeds<CHIP, LED_PIN_5, RGBORDER>(leds, curPos, hardware[i].count); break; \
                case LED_PIN_6: FastLED.addLeds<CHIP, LED_PIN_6, RGBORDER>(leds, curPos, hardware[i].count); break; \
                case LED_PIN_7: FastLED.addLeds<CHIP, LED_PIN_7, RGBORDER>(leds, curPos, hardware[i].count); break; \
                case LED_PIN_8: FastLED.addLeds<CHIP, LED_PIN_8, RGBORDER>(leds, curPos, hardware[i].count); break; \
                case LED_PIN_9: FastLED.addLeds<CHIP, LED_PIN_9, RGBORDER>(leds, curPos, hardware[i].count); break; \
                case LED_PIN_10: FastLED.addLeds<CHIP, LED_PIN_10, RGBORDER>(leds, curPos, hardware[i].count); break;

    #define ADD_LEDS_PIN_ERROR(i, ledPin) \
                        initializationError = String("In hardware slot ") + i + ": Led pin " + ledPin + " is not usable -- pins compiled in are " \
                            + LED_PIN_1 + ", " + LED_PIN_2 + ", " + LED_PIN_3 + ", " \
                            + LED_PIN_4 + ", " + LED_PIN_5 + ", " + LED_PIN_6 + ", " \
                            + LED_PIN_7 + ", " + LED_PIN_8 + ", " + LED_PIN_9 + ", " \
                            + LED_PIN_10; \
                        Serial.println(initializationError); \
                        hardware[i].isConfigured = false;

    #define ADD_LEDS_CHIP(CHIP) \
                if (hardware[i].rgbOrder.equals("RGB")) { \
                    switch (ledPin) { \
                        ADD_LEDS_PER_PIN(CHIP, RGB) \
                    default: \
                        ADD_LEDS_PIN_ERROR(i, ledPin) \
                        break; \
                    } \
                } else if (hardware[i].rgbOrder.equals("GRB")) { \
                    switch (ledPin) { \
                        ADD_LEDS_PER_PIN(CHIP, GRB) \
                    default: \
                        ADD_LEDS_PIN_ERROR(i, ledPin) \
                        break; \
                    } \
                } else { \
                    initializationError = String("In hardware slot ") + i + ": Unimplemented rgb order \"" \
                        + hardware[i].rgbOrder + "\""; \
                    Serial.println(initializationError); \
                    hardware[i].isConfigured = false; \
                }


            if (hardware[i].ledChip.equals("WS2811")) {
                ADD_LEDS_CHIP(WS2811)
            } else if (hardware[i].ledChip.equals("WS2812")) {
                ADD_LEDS_CHIP(WS2812)
            } else if (hardware[i].ledChip.equals("PL9823")) {
                ADD_LEDS_CHIP(PL9823)
            } else {
                initializationError = String("In hardware slot ") + i + ": Led chip " + hardware[i].ledChip
                    + " is not usable -- compiled chips are WS2811, WS2812, PL9823";
                Serial.println(initializationError);
                hardware[i].isConfigured = false;
            }
            if (hardware[i].isConfigured) {
                curPos += hardware[i].count;
            }
        }

//    .setCorrection(TypicalSMD5050)
//    .setTemperature(Tungsten100W);
;
//  FastLED.setDither();

        for (int i = 0; i < LED_HARDWARE_COUNT; i++) {
            if (!hardware[i].isConfigured) {
                continue;
            }
            digitalWrite(hardware[i].ledPin, LOW);
            pinMode(hardware[i].ledPin, OUTPUT);
        }

        FastLED.setBrightness(brightness);
        FastLED.setMaxPowerInVoltsAndMilliamps(5, powerLimit);

        FastLED.clear(false);
        FastLED.show();

        eventLoop->registerTimer(&timer);
        timer.setCallback([this](UEventLoopTimer *timer) { runOnce(); });
        if (isEnabled) {
            enable();
        }
    }

}

void LedService::enable() {
    avgFrameInterval = 0;
    avgShowDuration = 0;
    avgTotalDuration = 0;
    frame = 0;
    lastRunTm = micros();
    isEnabled = true;
    timer.setTimeout(intervalMillis);
}

void LedService::disable() {
    isEnabled = false;
    timer.cancelTimeout();
    FastLED.clear();
    FastLED.show();
}

void LedService::runOnce() {
    timer.setTimeout(intervalMillis);
    long tm1 = micros();

    if (frame > 0) {
        FastLED.show();
        long tmShow = micros();
        avgShowDuration = (avgShowDuration == 0 ? ((tmShow - tm1) << 4) : avgShowDuration * 63 / 64 + ((tmShow - tm1) << 4) / 64);
    }
    fill_solid(leds, totalLedCount, CRGB(0,0,0));

    for (int i = 0; i < controllerCount; i++) {
        controllers[i]->calc(frame);
    }

    long tmTotal = micros();
    avgTotalDuration = (avgTotalDuration == 0 ? ((tmTotal - tm1) << 4) : avgTotalDuration * 63 / 64 + ((tmTotal - tm1) << 4) / 64);

    uint32_t frameInterval = (uint32_t)(tm1 - lastRunTm);
    avgFrameInterval = (avgFrameInterval == 0 ? frameInterval : avgFrameInterval * 63 / 64 + frameInterval / 64);
    lastRunTm = tm1;

    if (frame % (1000 / intervalMillis) == 0) {
        Serial.print(":");
    }
    if (frame % ((1000 / intervalMillis) * 30) == 0) {
        Serial.printf("Avg duration micros: %lu; total %lu per led, %lu for showing, at %d fps (%d micros/frame)\n",
        (long)avgTotalDuration >> 4, (long)((avgTotalDuration / totalLedCount) >> 4), (long)((avgShowDuration / totalLedCount) >> 4),
        1000 / intervalMillis, intervalMillis * 1000);
        for (int i = 0; i < controllerCount; i++) {
            controllers[i]->printStats();
        }
    }

    ++frame;
}


#endif // USE_LED
