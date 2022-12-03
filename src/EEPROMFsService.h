#include <CompilationOpts.h>

#ifdef USE_EEPROM_LFS
#ifndef INCL_EEPROM_FS_SERVICE_H
#define INCL_EEPROM_FS_SERVICE_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "EEPROMDriver.h"
#include "EEPROMLittleFs.h"
#include "SystemService.h"

class EEPROMFsService {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr, SystemService *service);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;
    SystemService *systemService;
    ServiceCommands *cmd;

    bool isEnabled;
    int i2cPort;
    int i2cAddr;
    EEPROMLittleFs eepromLfs;
    bool isFsInitialized;

    void initCmd();
public:
    EEPROMLittleFs *getFs() { return isFsInitialized ? &eepromLfs : nullptr; }
};

#endif
#endif