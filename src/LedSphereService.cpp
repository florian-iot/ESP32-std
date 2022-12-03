#include <CompilationOpts.h>

#ifdef USE_LED_SPHERE

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LedSphereService.h"
#include "LogMgr.h"
#include "ButtonService.h"
#include "BeeperService.h"
#include "LedService.h"
#include "SystemService.h"

void LedSphereService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr,
    SystemService *systemService, ButtonService *buttonService, BeeperService *beeperService,
    LedService *ledService, OTA *ota)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("ledSphere");
    this->buttonService = buttonService;
    this->beeperService = beeperService;
    ota->onStart([this]() {
        String c("led disable");
        this->commandMgr->processCommandLine("ledSphere", &c);
        return true;
    });
    ota->onEnd([this](bool isSuccess) {
        String c("led enable");
        this->commandMgr->processCommandLine("ledSphere", &c);
        return true;
    });

    // defaults
    voltageSensePin = systemService->registerSysPin("ledSphere", "voltageSensePin");
    chargingSensePin = systemService->registerSysPin("ledSphere", "chargingSensePin");

    voltageDividerEnumerator = 2;
    voltageDividerDenominator = 1;
    batteryLowVoltage = 3700;
    batteryLowVoltageHyst = 200;
    usbPowerVoltage = 4400;
    usbPowerVoltageHyst = 100;
    isBatteryLow = false;
    isUsbPower = false;

    ServiceCommands *cmd = commandMgr->getServiceCommands("ledSphere");
    initDfa();
    initCommands(cmd);

    // load config
    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    // init data structures
    startupEnterCommands = nullptr;
    startupLeaveCommands = nullptr;
    clickCommands = nullptr;
    batteryLowEnterCommands = nullptr;
    batteryLowLeaveCommands = nullptr;
    usbPowerEnterCommands = nullptr;
    usbPowerLeaveCommands = nullptr;

    File f = SPIFFS.open("/ledSphereCommands.json", "r");
    JsonObject &commands = clickCommandsBuffer.parseObject(f);
    f.close();
    if (!commands.success()) {
        logger->error("Could not load json file ledSphereCommands.json");
    } else {
        Logger *logger = this->logger;
        auto parseCommands = [logger](JsonObject &commands, const char *key1, const char *key2) {
            if (commands.containsKey(key1)) {
                JsonObject &o = commands[key1].as<JsonObject>();
                if (o.success()) {
                    JsonArray *p = &o[key2].as<JsonArray>();
                    if (p->success()) {
                        String str;
                        p->printTo(str);
                        logger->debug("{}.{}: {}", LogValue(key1, LogValue::STATIC),
                            LogValue(key2, LogValue::STATIC), LogValue(str.c_str(), LogValue::DO_COPY));
                        return p;
                    } else {
                        logger->error("Could not parse \"{}.{}\" in ledSphereCommands.json",
                            LogValue(key1, LogValue::STATIC), LogValue(key2, LogValue::STATIC));
                    }
                } else {
                    logger->error("Could not parse \"{}\" in ledSphereCommands.json", LogValue(key1, LogValue::STATIC));
                }
            }
            return (JsonArray *)nullptr;
        };

        startupEnterCommands = parseCommands(commands, "startup", "enter");
        startupLeaveCommands = parseCommands(commands, "startup", "leave");
        batteryLowEnterCommands = parseCommands(commands, "batteryLow", "enter");
        batteryLowLeaveCommands = parseCommands(commands, "batteryLow", "leave");
        usbPowerEnterCommands = parseCommands(commands, "usbPower", "enter");
        usbPowerLeaveCommands = parseCommands(commands, "usbPower", "leave");

        JsonArray *p = &commands["click"].as<JsonArray>();
        if (!p->success()) {
            logger->error("Could not parse \"click\" in ledSphereCommands.json");
        } else {
            // all elements of p are arrays of strings
            bool isOk = true;
            for (int i = 0; i < p->size(); i++) {
                if (!(*p)[i].is<JsonArray>()) {
                    logger->error("Could not parse \"click[{}]\" as an array of strings in ledSphereCommands.json", i);
                    isOk = false;
                }
            }
            if (isOk) {
                String str;
                p->printTo(str);
                logger->debug("click: {}", LogValue(str.c_str(), LogValue::DO_COPY));
                clickCommands = p;
            }
        }
    }

    if (startupEnterCommands == nullptr) {
        startupEnterCommands = &clickCommandsBuffer.createArray();
    }
    if (startupLeaveCommands == nullptr) {
        startupLeaveCommands = &clickCommandsBuffer.createArray();
    }
    if (clickCommands == nullptr) {
        clickCommands = &clickCommandsBuffer.createArray();
    }
    if (batteryLowEnterCommands == nullptr) {
        batteryLowEnterCommands = &clickCommandsBuffer.createArray();
    }
    if (batteryLowLeaveCommands == nullptr) {
        batteryLowLeaveCommands = &clickCommandsBuffer.createArray();
    }
    if (usbPowerEnterCommands == nullptr) {
        usbPowerEnterCommands = &clickCommandsBuffer.createArray();
    }
    if (usbPowerLeaveCommands == nullptr) {
        usbPowerLeaveCommands = &clickCommandsBuffer.createArray();
    }

    clickIndex = 0;

    auto button = buttonService->getButton(0);
    if (button) {
        button->onEvent(ButtonService::BUTTON_CLICK,
                [this](int id, ButtonService::ButtonEvent event, bool alreadyHandled) {
            logger->debug("Cliked!");
            dfa.handleInput(CLICK);
            return true;
        });
    } else {
        logger->info("Button 0 is not configured, check command \"button\" and \"button0\"");
    }

    // init hardware
    if (voltageSensePin > 0) {
        pinMode(voltageSensePin, INPUT);
        uint32_t v = analogReadMilliVolts(voltageSensePin);
        if (v >= 0x7FFFFFFF) {
            avgBatteryVoltage = 3800;
        } else {
            avgBatteryVoltage = v * voltageDividerEnumerator / voltageDividerDenominator;
        }
    } else {
        avgBatteryVoltage = 3800; // assuming normal battery voltage
        logger->info("Voltage sensing pin is not configured");
    }
    if (chargingSensePin > 0) {
        pinMode(chargingSensePin, INPUT_PULLUP);
        // TODO
        // batteryChargingStatus = CHARGING_NONE;
        // batteryChargingLevel = LOW;
        // batteryChargingHighCount = 0;
        // batteryChargingLowCount = 0;
    } else {
        logger->info("Battery charging sensing pin is not configured");
    }

    batteryVoltageTimer.init(eventLoop, [this](UEventLoopTimer *timer) {
        if (voltageSensePin > 0) { // maybe the pins are configured manually later
            voltageCheck();
        }
        if (chargingSensePin > 0) {
            chargingCheck();
        }
    });
    batteryVoltageTimer.setInterval(250); // check 4 times per second

    dfa.handleInput(START);
}

void LedSphereService::initDfa()
{
    dfa.init(eventLoop, nullptr /* or logger, to log */, LS_STARTUP);

    dfa.onInput([this](Dfa *dfa, Dfa::State state, Dfa::Input input) {

        if (state.is(LS_STARTUP)) {

            if (input.is(START)) {
                if (isBatteryLow) {
                    return dfa->transitionTo(LS_BATTERY_LOW);
                } else if (isUsbPower) {
                    return dfa->transitionTo(LS_ON_USB_STANDBY);
                } else {
                    return dfa->transitionTo(LS_START_SEQUENCE);
                }
            }

        } else if (state.is(LS_START_SEQUENCE)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                // load initial led config
                for (int i = 0; i < startupEnterCommands->size(); i++) {
                    String cmd = (*startupEnterCommands)[i].as<const char *>();
                    this->commandMgr->processCommandLine("ledSphere", &cmd);
                }
                String cmd("led0 sparkDensityPct");
                this->commandMgr->processCommandLine("ledSphere", &cmd);
                dfaStartSparkDensityPct = cmd.toInt();
                dfaStartSeq = 0;

                cmd = "led0 sparkDensityPct 1";
                this->commandMgr->processCommandLine("ledSphere", &cmd);

                dfa->setStateTimeout(500);
                return dfa->noTransition();
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                if (dfaStartSeq <= 10) {
                    int d = dfaStartSeq * dfaStartSparkDensityPct / 10;
                    if (d == 0) {
                        d = 1;
                    }
                    String cmd = "led0 sparkDensityPct "; cmd.concat(d);
                    this->commandMgr->processCommandLine("ledSphere", &cmd);
                    ++dfaStartSeq;
                    dfa->setStateTimeout(1000);
                } else {
                    for (int i = 0; i < startupLeaveCommands->size(); i++) {
                        String cmd = (*startupLeaveCommands)[i].as<const char *>();
                        this->commandMgr->processCommandLine("ledSphere", &cmd);
                    }
                    clickIndex = 0;
                    return dfa->transitionTo(LS_ACTIVE); // not to LS_START_ACTIVE, as we don't want to re-apply clickCommands[0]
                }
            } else if (input.is(BATTERY_LOW, USB_POWER)) {
                for (int i = 0; i < startupLeaveCommands->size(); i++) {
                    String cmd = (*startupLeaveCommands)[i].as<const char *>();
                    this->commandMgr->processCommandLine("ledSphere", &cmd);
                }
                return dfa->transitionTo(input.is(BATTERY_LOW) ? LS_BATTERY_LOW : LS_ON_USB_STANDBY);
            } else if (input.is(CLICK)) {
                this->beeperService->beep(6, 9, 10);
                return dfa->noTransition();
            }

        } else if (state.is(LS_START_ACTIVE)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                for (int i = 0; i < (*clickCommands)[clickIndex].size(); i++) {
                    String cmd = (*clickCommands)[clickIndex][i].as<const char *>();
                    this->commandMgr->processCommandLine("ledSphere", &cmd);
                }
                return dfa->transitionTo(LS_ACTIVE);
            }

        } else if (state.is(LS_ACTIVE)) {

            if (input.is(CLICK)) {
                if (isUsbPower) {
                    return dfa->transitionTo(LS_ON_USB_STANDBY);
                } else {
                    ++clickIndex;
                    if (clickIndex > clickCommands->size()) {
                        clickIndex = 0;
                    }
logger->debug("Click index: {}, commands: {}", clickIndex, (*clickCommands)[clickIndex].size());
                    for (int i = 0; i < (*clickCommands)[clickIndex].size(); i++) {
                        String cmd = (*clickCommands)[clickIndex][i].as<const char *>();
                        this->commandMgr->processCommandLine("ledSphere", &cmd);
                    }
                    this->beeperService->beep(6, 2, 30, 0, 6, 9, 30);
                    return dfa->noTransition();
                }
            } else if (input.is(BATTERY_LOW, USB_POWER)) {
                return dfa->transitionTo(input.is(BATTERY_LOW) ? LS_BATTERY_LOW : LS_ON_USB_STANDBY);
            }

        } else if (state.is(LS_BATTERY_LOW)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                for (int i = 0; i < batteryLowEnterCommands->size(); i++) {
                    String cmd = (*batteryLowEnterCommands)[i].as<const char *>();
                    this->commandMgr->processCommandLine("ledSphere", &cmd);
                }
                return dfa->noTransition();
            } else if (input.is(BATTERY_LOW_TERMINATED, USB_POWER)) {
                for (int i = 0; i < batteryLowLeaveCommands->size(); i++) {
                    String cmd = (*batteryLowLeaveCommands)[i].as<const char *>();
                    this->commandMgr->processCommandLine("ledSphere", &cmd);
                }
                return dfa->transitionTo(input.is(USB_POWER) ? LS_ON_USB_STANDBY : LS_START_ACTIVE);
            } else if (input.is(CLICK)) {
                this->beeperService->beep(5, 2, 100);
                return dfa->noTransition();
            }

        } else if (state.is(LS_ON_USB_STANDBY)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                String cmd;
                for (int i = 0; i < usbPowerEnterCommands->size(); i++) {
                    cmd = (*usbPowerEnterCommands)[i].as<const char *>();
                    this->commandMgr->processCommandLine("ledSphere", &cmd);
                }
                this->beeperService->beep(6, 2, 20, 0, 6, 3, 20, 0, 6, 5, 20, 0, 6, 7, 20, 0, 6, 8, 20);
                return dfa->noTransition();
            } else if (input.is(BATTERY_LOW, USB_POWER_TERMINATED, CLICK)) {
                String cmd;
                for (int i = 0; i < usbPowerLeaveCommands->size(); i++) {
                    cmd = (*usbPowerLeaveCommands)[i].as<const char *>();
                    this->commandMgr->processCommandLine("ledSphere", &cmd);
                }
                if (input.is(BATTERY_LOW)) {
                    return dfa->transitionTo(LS_BATTERY_LOW);
                } else if (input.is(USB_POWER_TERMINATED)) {
                    this->beeperService->beep(6, 8, 20, 0, 6, 7, 20, 0, 6, 5, 20, 0, 6, 3, 20, 0, 6, 2, 20);
                    return dfa->transitionTo(LS_START_SEQUENCE);
                } else { // CLICK
                    ++clickIndex;
                    if (clickIndex > clickCommands->size()) {
                        clickIndex = 0;
                    }
                    this->beeperService->beep(6, 2, 30, 0, 6, 9, 30);
                    return dfa->transitionTo(LS_START_ACTIVE);
                }
            }

        }

        return dfa->transitionError();

    });
}

void LedSphereService::initCommands(ServiceCommands *cmd)
{
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&voltageSensePin)
        .help("--> Pin for measuring voltage")
        .setFn([this](int val, bool isLoading, String *msg) {
            if (!isLoading && voltageSensePin > 0) {
                pinMode(voltageSensePin, INPUT);
            }
            voltageSensePin.setPin(val);
            pinMode(voltageSensePin, INPUT);
            return true;
        })
    );

    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&chargingSensePin)
        .help("--> Pin for sensing charging status")
        .setFn([this](int val, bool isLoading, String *msg) {
            if (!isLoading && voltageSensePin > 0) {
                pinMode(chargingSensePin, INPUT);
            }
            chargingSensePin.setPin(val);
            pinMode(chargingSensePin, INPUT_PULLUP);
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("voltageDividerEnumerator", true)
        .cmd("voltageDividerEnumerator")
        .help("--> Enumerator for the voltage divider")
        .vMin(1)
        .vMax(500000) // so that 4096 * enumerator remains within uint32_t
        .ptr(&voltageDividerEnumerator)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("voltageDividerDenominator", true)
        .cmd("voltageDividerDenominator")
        .help("--> Denominator for the voltage divider")
        .vMin(1)
        .vMax(500000)
        .ptr(&voltageDividerDenominator)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("lowVoltage", true)
        .cmd("lowVoltage")
        .help("--> Battery low voltage level, in millivolts")
        .ptr(&batteryLowVoltage)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("batteryLowVoltageHyst", true)
        .cmd("batteryLowVoltageHyst")
        .vMin(0)
        .vMax(1000)
        .help("--> Voltage rise needed from batteryLowVoltage to be considered as out of low voltage condition, in millivolts")
        .ptr(&batteryLowVoltageHyst)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("usbPowerVoltage", true)
        .cmd("usbPowerVoltage")
        .help("--> Voltage to consider as USB power, in millivolts")
        .ptr(&usbPowerVoltage)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("usbPowerVoltageHyst", true)
        .cmd("usbPowerVoltageHyst")
        .help("--> Voltage drop needed from usbPowerVoltage to be considered as out of USB power, in millivolts")
        .vMin(0)
        .vMax(1000)
        .ptr(&usbPowerVoltageHyst)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("voltage", true)
        .cmd("voltage")
        .help("--> Input voltage, battery or USB, in millivolts")
        .isPersistent(false)
        .getFn([this]() {
            return (int)avgBatteryVoltage;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("dfaState", true)
        .cmd("dfaState")
        .help("--> DFA state")
        .isPersistent(false)
        .getFn([this](String *msg) {
            *msg = dfa.stateName(dfa.getState());
            if (dfa.getStateTimeout() > 0) {
                *msg += String(", timeout ") + dfa.getStateTimeout()
                    + " ms, spent " + String((int32_t)dfa.getMillisInState()) + " ms";
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("clickIndex", true)
        .cmd("clickIndex")
        .help("--> Index in click commands")
        .isPersistent(false)
        .getFn([this]() {
            return clickIndex;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("simulateClick", true)
        .cmdOn("simulateClick")
        .helpOn("--> Simulate a click")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            logger->debug("Simulating a click!");
            dfa.queueInput(CLICK, 1);
            return true;
        })
    );

}

void LedSphereService::voltageCheck()
{
    uint32_t v = analogReadMilliVolts(voltageSensePin);
    if (v >= 0x7FFFFFFF) {
        return;
    }
    v = v * voltageDividerEnumerator / voltageDividerDenominator;

    avgBatteryVoltage += ((int32_t)v - (int32_t)avgBatteryVoltage + 2) / 4;

    bool batteryTriggered = false;
    bool usbTriggered = false;

    if (isBatteryLow) {
        if (avgBatteryVoltage >= batteryLowVoltage + batteryLowVoltageHyst) {
            isBatteryLow = false;
            batteryTriggered = true;
        }
        if (avgBatteryVoltage >= usbPowerVoltage) {
            isUsbPower = true;
            usbTriggered = true;
        }
    } else if (!isBatteryLow && !isUsbPower) {
        if (avgBatteryVoltage < batteryLowVoltage) {
            isBatteryLow = true;
            batteryTriggered = true;
        }
        if (avgBatteryVoltage >= usbPowerVoltage) {
            isUsbPower = true;
            usbTriggered = true;
        }
    } else if (isUsbPower) {
        if (avgBatteryVoltage < batteryLowVoltage) {
            isBatteryLow = true;
            batteryTriggered = true;
        }
        if (avgBatteryVoltage < usbPowerVoltage - usbPowerVoltageHyst) {
            isUsbPower = false;
            usbTriggered = true;
        }
    }

    if (batteryTriggered && isBatteryLow) {
        logger->debug("Battery low, voltage: {}", avgBatteryVoltage);
        dfa.handleInput(BATTERY_LOW);
    } else if (batteryTriggered && !isBatteryLow) {
        if (usbTriggered && isUsbPower) {
            logger->debug("On USB power after battery low, voltage: {}", avgBatteryVoltage);
            dfa.handleInput(USB_POWER);
        } else {
            logger->debug("Battery low terminated, voltage: {}", avgBatteryVoltage);
            dfa.handleInput(BATTERY_LOW_TERMINATED);
        }
    } else if (usbTriggered && isUsbPower) {
        logger->debug("On USB power, voltage: {}", avgBatteryVoltage);
        dfa.handleInput(USB_POWER);
    } else if (usbTriggered && !isUsbPower) {
        logger->debug("No longer on USB power, battery normal, voltage: {}", avgBatteryVoltage);
        dfa.handleInput(USB_POWER_TERMINATED);
    }
}

void LedSphereService::chargingCheck()
{
    uint32_t v = analogReadMilliVolts(chargingSensePin);
    if (v >= 0x7FFFFFFF) {
        return;
    }

    if (v < 2800) { // led turned on

    } else if (v > 2900) { // led turned off

    } // else we ignore

}
#endif
