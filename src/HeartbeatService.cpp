#include <CompilationOpts.h>

#ifdef USE_HEARTBEAT

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "HeartbeatService.h"
#include "LogMgr.h"

void HeartbeatService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("heartbeat");

    ServiceCommands *cmd = commandMgr->getServiceCommands("heartbeat");
    initDfa();
    initCommands(cmd);

    // defaults

#ifdef HEARTBEAT_PIN
    ledPin = HEARTBEAT_PIN;
#else
    ledPin = -1;
#endif
    isEnabled = true;
    isSerialDot = true;
    period = 1000;
    isInverted = false;

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
    cnt = 0;
    index = 0;
    for (int i = 0; i < 4; i++) {
        durations[i] = ((int[]){ 100, 5, 1390, 5 })[i];
    }

    // init hardware
    if (ledPin != -1) {
        digitalWrite(ledPin, LOW);
        pinMode(ledPin, OUTPUT);
    }

}

void HeartbeatService::initDfa()
{
    dfa.init(eventLoop, nullptr /* or logger, to log */, HB_STARTUP);

    dfa.onInput([this](Dfa *dfa, Dfa::State state, Dfa::Input input) {

        if (state.is(HB_STARTUP) && input.is(Dfa::Input::ENTER_STATE)) {

            if (isEnabled) {
                return dfa->transitionTo(HB_DO_ENABLE);
            } else {
                return dfa->transitionTo(HB_DISABLED);
            }

        } else if (state.is(HB_DISABLED) && input.is(ENABLE)) {

            return dfa->transitionTo(HB_DO_ENABLE);

        } else if (state.is(HB_DO_ENABLE) && input.is(Dfa::Input::ENTER_STATE)) {

            index = 0;
            if (ledPin != -1) {
                digitalWrite(ledPin, LOW);
                pinMode(ledPin, OUTPUT);
            }
            isEnabled = true;
            return dfa->transitionTo(HB_ENABLED);

        } else if (state.is(HB_ENABLED) && input.is(Dfa::Input::ENTER_STATE)) {

            index = 0;
            return dfa->transitionTo(HB_ENABLED, 1); // could have set timeout while transitioning to this state, but here for testing

        } else if (state.is(HB_ENABLED) && input.is(DISABLE)) {

            if (ledPin != -1) {
                pinMode(ledPin, INPUT);
            }
            isEnabled = false;
            return dfa->transitionTo(HB_DISABLED);

        } else if (state.is(HB_ENABLED) && input.is(Dfa::Input::TIMEOUT)) {

            ++cnt;
            if (isSerialDot) {
                if ((cnt & 0x3F) == 0) {
                    Serial.printf(". Count = %ld\n", cnt);
                } else {
                    Serial.printf(".");
                }
            }

            ++index;
            if (index == sizeof(durations) / sizeof(durations[0])) {
                index = 0;
            }
            if (ledPin != -1) {
                digitalWrite(ledPin, (index & 0x01) ^ isInverted);
            }

            return dfa->transitionTo(HB_ENABLED, durations[index] * period / 1000);
        } else {
            return dfa->transitionError();
        }
    });
}

void HeartbeatService::initCommands(ServiceCommands *cmd)
{
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("ledPin", true)
        .cmd("ledPin")
        .help("--> Set led pin, set to -1 for no led")
        .vMin(-1)
        .vMax(99)
        .ptr(&ledPin)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            if (isLoading) {
                ledPin = val;
            } else {
                if (ledPin != -1) {
                    digitalWrite(ledPin, LOW);
                    pinMode(ledPin, INPUT);
                }
                ledPin = val;
                if (ledPin != -1) {
                    digitalWrite(ledPin, LOW);
                    pinMode(ledPin, OUTPUT);
                }
            }
            *msg = "Led pin set to "; *msg += ledPin;
            return true;
        })
    );
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("inverted", true)
        .cmd("inverted")
        .help("--> Set to true if the led inverted, i.e., is on when the signal is low")
        .ptr(&isInverted)
    );
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("serial", true)
        .cmd("serial")
        .help("--> Enables or disables printing to serial link a \".\" at each iteration")
        .ptr(&isSerialDot)
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("enabled", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .helpOn("--> Enables the service")
        .helpOff("--> Disables the service")
        .ptr(&isEnabled)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (isLoading) {
                isEnabled = val;
                return true;
            }

            bool processed = dfa.handleInput(val ? ENABLE : DISABLE);
            if (val) {
                *msg = (processed ? "Enabled" : "Already enabled");
            } else {
                *msg = (processed ? "Disabled" : "Already disabled");
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("period", true)
        .cmd("period")
        .help("--> Blink period, in millis (default 1000)")
        .ptr(&period)
    );

}

#endif
