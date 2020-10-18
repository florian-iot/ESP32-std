#include <CompilationOpts.h>

#ifdef USE_LM75A_TEMP

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LM75A_TempService.h"

void LM75A_TempService::init(UEventLoop *eventLoop, CommandMgr *commandMgr)
{
  this->eventLoop = eventLoop;
  this->commandMgr = commandMgr;

  ServiceCommands *cmd = commandMgr->getServiceCommands("temp");

  cmd->registerIntData(
    ServiceCommands::IntDataBuilder("i2cAddress", true)
    .cmd("i2cAddress")
    .help("--> Set the I2C address as set by pins A0, A1, A2, between 72 (0x48) and 79 (0x4F). Disables the sensor (it must be re-enabled).")
    .vMin(0x48)
    .vMax(0x4f)
    .ptr(&i2cAddress)
    .setFn([this](int val, bool isLoading, String *msg) -> bool {
      if (isEnabled) {
        disable();
      }
      i2cAddress = val;
      *msg = "I2C address set to "; *msg += i2cAddress; *msg += ", sensor is disabled";
      return true;
    })
  );

  cmd->registerBoolData(
    ServiceCommands::BoolDataBuilder("read", true)
    .isPersistent(false)
    .cmdOn("read")
    .helpOn("--> Read the current temperature")
    .setFn([this](int val, bool isLoading, String *msg) -> bool {
      if (isEnabled && lm75a != nullptr) {
        float t = lm75a->getTemperature();
        if (t != LM75A_INVALID_TEMPERATURE) {
          *msg = String(t);
        } else {
          *msg = "Temperature reading failed";
        }
      } else {
        *msg = "Sensor is not enabled";
      }
      return true;
    })
  );

  cmd->registerBoolData(
    ServiceCommands::BoolDataBuilder("enabled", true)
    .cmdOn("enable")
    .cmdOff("disable")
    .helpOn("--> Enables the sensor")
    .helpOff("--> Disables the sensor")
    .ptr(&isEnabled)
    .setFn([this](bool val, bool isLoading, String *msg) -> bool {
      if (isLoading) {
        isEnabled = true;
      } else {
        if (val == isEnabled) {
          // nothing to do
          *msg = "Sensor is already ";
          *msg += (val ? "enabled" : "disabled");
          return true;
        }
        if (val) {
          enable();
          if (isLogging) {
            startLogging();
          }
        } else {
          disable();
        }
        *msg = "Temperature sensor is ";
        *msg += (isEnabled ? "enabled" : "disabled");
      }
      return true;
    })
  );

  cmd->registerBoolData(
    ServiceCommands::BoolDataBuilder("log", true)
    .cmd("log")
    .help("log on|off --> enables temperature logging to serial")
    .ptr(&isLogging)
    .setFn([this](bool val, bool isLoading, String *msg) -> bool {
      if (isLoading) {
        isLoading = val;
      } else {
        if (val == isLogging) {
          // nothing to do
          *msg = "Logging is already ";
          *msg += (val ? "on" : "off");
          return true;
        }
        isLogging = val;
        if (isLogging) {
          startLogging();
        }
        *msg = "Logging is ";
        *msg += (isLogging ? "on" : "off");
      }
      return true;
    })
  );

  cmd->onAfterLoad([this](String *msg) {
    if (isEnabled) {
      enable();
      if (isLogging) {
        startLogging();
      }
    }
  });

  eventLoop->registerTimer(&loggingTimer);

  // defaults
  this->i2cAddress = LM75A_TEMP_I2CADDRESS;
  this->isEnabled = true;
  this->isLogging = true;
  this->isHardwareOk = false; // will be ok once we set it up

  String msg;
  bool rc;
  rc = cmd->load(nullptr, &msg);
  if (rc) {
    String keyName;
    cmd->getCurrentKeyName(&keyName);
    Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
  }

}

void LM75A_TempService::enable()
{
  if (!isHardwareOk) {
    lm75a = new M2M_LM75A(i2cAddress, &Wire);
    isHardwareOk = lm75a->isConnected();
    if (!isHardwareOk) {
      Serial.println("LM57A hardware is not connected, temperature service disabled");
      delete lm75a;
      lm75a = nullptr;
    } else {
      uint8_t prodId = lm75a->getProdIdAsInt();
      Serial.println(String("LM57A hardware connected and enabled, prod ID: ") + String(prodId, HEX));
    }
  }
  if (isHardwareOk) {
    lm75a->wakeup();
    isEnabled = true;
  } else {
    isEnabled = false;
  }
}

void LM75A_TempService::disable()
{
  if (!isHardwareOk || !isEnabled) {
    isEnabled = false;
    return;
  }
  if (lm75a != nullptr) {
    lm75a->shutdown();
    delete lm75a;
    lm75a = nullptr;
  }
  isEnabled = false;
}

void LM75A_TempService::startLogging()
{
  if (loggingTimer.isActive()) {
    return;
  }
  loggingTimer.setTimeout([this](UEventLoopTimer *timer) {
    if (!isLogging || !isEnabled) {
      return; // timer will not be set for next time
    }
    float t = lm75a->getTemperature();
    Serial.printf("Temperature: %.03f °C\n", t);
    loggingTimer.setTimeout(5000);
  }, 1L);
}


#endif
