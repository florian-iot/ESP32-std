
#include <CompilationOpts.h>

#define USE_TIME
#ifdef USE_TIME

#include "TimeService.h"
#include <stdlib.h>
#include <Arduino.h>
#include "sys/time.h"
#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"


void TimeService::init(UEventLoop *eventLoop, CommandMgr *commandMgr)
{
  this->eventLoop = eventLoop;
  this->commandMgr = commandMgr;

  eventLoop->registerTimer(&syncTimer);
  syncTimer.setTimeout([this](UEventLoopTimer *timer) {
    // do the necessary... TODO
    if (isDebug) {
        Serial.println("Syncing time...");
    }

    timer->setTimeout(10 * 60 * 1000L); // synchronize every 10 minutes

  }, 1L);


    ServiceCommands *cmd = commandMgr->getServiceCommands("time");
    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("timezone", true)
            .cmd("timezone")
            .help("--> Set TZ environment variable, defaults to " TIME_TZ_INFO " (more info here: https://remotemonitoringsystems.ca/time-zone-abbreviations.php)")
            .ptr(&tzInfo)
            .setFn([this](const String &val, bool isLoading, String *msg) -> bool {
                tzInfo = val;
                setenv("TZ", tzInfo.c_str(), 1);
                *msg = "Time zone set to "; *msg += tzInfo;
                return true;
            })
    );
    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("ntpserver", true)
            .cmd("ntpserver")
            .help("--> Set NTP server, NONE for not using NTP (defaults to " TIME_NTP_SERVER)
            .ptr(&ntpServer)
            .setFn([this](const String &val, bool isLoading, String *msg) -> bool {
                ntpServer = val;
                *msg = "NTP server set to"; *msg += ntpServer;
                return true;
            })
    );

    // defaults
    isDebug = true;
    tzInfo = TIME_TZ_INFO;
    ntpServer = TIME_NTP_SERVER;
    isUseRtc = true;

    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    lastSyncedTime = 0; // as of now, we're at 1970-01-01
    lastSyncedMillis = millis();
    isSynced = false;
}



time_t TimeService::getNow()
{
    return lastSyncedTime + (int)(millis() - lastSyncedMillis) / 1000;
}

void TimeService::sync()
{
    if (this->isUseRtc) {

    }
}





#endif
