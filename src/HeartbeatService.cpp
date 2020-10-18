#include <CompilationOpts.h>

#ifdef USE_HEARTBEAT

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "HeartbeatService.h"

void HeartbeatService::init(UEventLoop *eventLoop, CommandMgr *commandMgr)
{
  this->eventLoop = eventLoop;
  this->commandMgr = commandMgr;
  cnt = 0;
  state = 0;
  for (int i = 0; i < 4; i++) {
    durations[i] = ((int[]){ 250, 250, 1000, 1000 })[i];
  }

  isEnabled = true;
  isSerialDot = true;
#ifdef HEARTBEAT_PIN
  ledPin = HEARTBEAT_PIN;
#else
  this->ledPin = -1;
#endif

  if (ledPin != -1) {
    digitalWrite(ledPin, LOW);
    pinMode(ledPin, OUTPUT);
  }

  eventLoop->registerTimer(&heartbeatTimer);
  heartbeatTimer.setTimeout([this](UEventLoopTimer *timer) {
    if (!isEnabled) {
      return;
    }

    ++cnt;

    if (isSerialDot) {
      if ((cnt & 0x3F) == 0) {
        Serial.printf(". Count = %ld\n", cnt);
      } else {
        Serial.printf(".");
      }
    }

    ++state;
    if (state == sizeof(durations) / sizeof(durations[0])) {
      state = 0;
    }
    if (ledPin != -1) {
      digitalWrite(ledPin, state & 0x01);
    }
    timer->setTimeout(durations[state]);
  }, 1L);

  ServiceCommands *cmd = commandMgr->getServiceCommands("heartbeat");
  cmd->registerIntData(
    ServiceCommands::IntDataBuilder("ledPin", true)
    .cmd("ledPin")
    .help("--> Set led pin, set to -1 for no led")
    .vMin(-1)
    .vMax(99)
    .ptr(&ledPin)
    .setFn([this](int val, bool isLoading, String *msg) -> bool {
      if (ledPin != -1) {
        digitalWrite(ledPin, LOW);
        pinMode(ledPin, INPUT);
      }
      ledPin = val;
      if (ledPin != -1) {
        digitalWrite(ledPin, LOW);
        pinMode(ledPin, OUTPUT);
      }
      *msg = "Led pin set to "; *msg += ledPin;
      return true;
    })
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
      if (val) {
        if (!isEnabled) {
          isEnabled = true;
          heartbeatTimer.setTimeout(1); // start immediately
          *msg = "Enabled";
        } else {
          *msg = "Already enabled";
        }
      } else {
        if (isEnabled) {
          isEnabled = false;
          if (ledPin != -1) {
            digitalWrite(ledPin, LOW);
          }
          *msg = "Disabled";
        } else {
          *msg = "Already disabled";
        }
      }
      return true;
    })
  );

  String msg;
  bool rc;
  rc = cmd->load(nullptr, &msg);
  if (rc) {
    String keyName;
    cmd->getCurrentKeyName(&keyName);
    Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
  }
}


#endif
