#include <CompilationOpts.h>

#ifdef USE_INA3221
#ifndef INCL_INA3221_H
#define INCL_INA3221_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "SystemService.h"

class Ina3221Service {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr, SystemService *systemService);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;

    bool isEnabled;
    i2c_port_t i2cPort;
    int i2cAddress;
    bool channelEnabled[3];
    int shuntResistance[3]; // milliohms
    uint8_t currentReg;
    String lastError;
    int loggingIntervalMillis;
    UEventLoopTimer timer;

    i2c_cmd_handle_t cmd; // used for i2c communication, deleted lazily to save some time

    void initCommands(ServiceCommands *cmd);
    bool setupDeviceNormal();
    // channel: 1..3; sets up for checking current on channel as quickly as possible
    bool setupDeviceCheckOvercurrent(int channel);
    bool identify();
    bool readRegister(uint8_t register, uint16_t *value);
    bool writeRegister(uint8_t register, uint16_t value);
public:
    // millivolts and milliamps
    bool readChannelData(int channel, int *voltage, int *current);
    bool setModeOvercurrent(int channel);
    bool setModeNormal();
};

#endif
#endif