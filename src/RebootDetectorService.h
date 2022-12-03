#include <CompilationOpts.h>

#ifdef USE_REBOOT_DETECTOR
#ifndef INCL_REBOOT_DETECTOR_H
#define INCL_REBOOT_DETECTOR_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "SystemService.h"

class RebootDetectorService {
public:
    RebootDetectorService();
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr, SystemService *systemService, FS *fs);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;
    FS *fs;
    SystemService *systemService;

    bool isEnabled;
    const char *fileName;
    UEventLoopTimer timer;
    int quickRebootsCount;
    bool involuntaryShutdown;

    void initCommands(ServiceCommands *cmd);

public:
    enum Level {
        APPLICATION = 2,
        INFRASTRUCTURE = 1,
        MINIMUM = 0
    };

    bool isActive();
    Level getRebootLevel();
    const char *getRebootLevelStr();
    void resetRebootLevel();
    bool isInvoluntaryShutdown();
};

#endif
#endif