#include <CompilationOpts.h>
#ifdef USE_PCF8563
#ifndef INCL_RTC_PCF8563_H
#define INCL_RTC_PCF8563_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "SystemService.h"

#include "pcf8563.h"

class RtcPcf8563Service {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr, SystemService *systemService);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    SystemService *systemService;
    Logger *logger;

    UEventLoopTimer  heartbeatTimer;
    void initCommands(ServiceCommands *cmd);

    bool isEnabled;
    int i2cPort;
    int i2cAddr;
    PCF8563_Class rtc;
    bool isAutoSyncSystemTime;
    String tzInfo;

    void setSystemTime();
    void getSystemTime(String *msg);
    void getRtcTime(String *msg);
    time_t rtcTime();
    void getRtcTimeLocal(String *msg);

public:
    void clockDisable();
    /**
     * freq is one of 1, 32, 1024000, 32768000
     */
    bool clockEnable(int freq);
    void timerDisable();

    enum TIMER_ROUND_DIRECTION {
        TIMER_ROUND_UP,
        TIMER_ROUND_DOWN
    };
    /**
     * Returns the actual duration of the timer.
     * Can be different because of rounding and different frequencies available.
     * If more than maximum duration is requested (256 minutes), then the maximum
     * duration is set.
     */
    uint32_t timerEnable(int millis, TIMER_ROUND_DIRECTION dir);

};

#endif
#endif