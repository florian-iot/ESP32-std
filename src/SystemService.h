#include <CompilationOpts.h>

#ifndef INCL_SYSTEM_SERVICE_H
#define INCL_SYSTEM_SERVICE_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <driver/i2c.h>
#include <Wire.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "Util.h"
#include "System.h"

class SystemService {
public:
    SystemService();
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;

    // Time since reboot: rebootTimeMillis + (millis() - rebootTimeTs)
    uint64_t rebootTimeMillis;
    uint32_t rebootTimeTs; // time at which rebootTimeMillis is taken.
    UEventLoopTimer uptimeTimer;
    bool isMustReboot;
    bool isShuttingDown;
    CallbackList<void ()> shutdownCallbacks;

#ifdef USE_SYSTEM_I2C
    int i2c_sda[2];
    int i2c_scl[2];
    int i2c_freq[2];
    enum I2cStatus {
        NOT_USED,
        IDF,
        TWO_WIRE
    };
    I2cStatus i2c_status[2];
    int i2c_useCount[2];

    void initI2cCommands(ServiceCommands *cmd);
    void scan(int port, String *msg);
#endif

    std::vector<SysPinData*> sysPins;

public:
    void getInfo(String *info);
    void getResetReason(String *info);
    bool isResetReasonUnexpected();
    uint64_t uptime();
    bool mustReboot();
    void setMustReboot();
    void setWillShutDown();
    int onShutdown(std::function<void ()>fn);
    void removeOnShutdown(int handle);

#ifdef USE_SYSTEM_I2C
    /** returns null if the port was already used via getIdf() */
    TwoWire *getTwoWire(int port);
    void releaseTwoWire(int port);
    bool getIdf(i2c_port_t port);
    bool getIdf(int port);
    void releaseIdf(i2c_port_t port);
    void releaseIdf(int port);
#endif

    SysPin registerSysPin(const char *service, const char *name);

};

#endif
