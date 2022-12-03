#include <CompilationOpts.h>

#ifdef USE_AM2320
#ifndef INCL_AM2320_H
#define INCL_AM2320_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "SystemService.h"
#include <driver/i2c.h>

class AM2320Service {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, SystemService *systemService, LogMgr *logMgr);

    void startMeasuring();
    bool isMeasuringDone();
    void getMeasures(int *envTemp, int *envHumidity);

private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;

    Dfa dfa;

    Dfa::Input START_MEASURE = dfa.nextInput("START_MEASURE");

    Dfa::State STARTUP = dfa.nextState("STARTUP");
    Dfa::State IDLE = dfa.nextState("IDLE");
    Dfa::State MEASURE_1 = dfa.nextState("MEASURE_1");
    Dfa::State MEASURE_2 = dfa.nextState("MEASURE_2");

    void initCommands(ServiceCommands *cmd);
    void initDfa();

    bool isEnabled;
    i2c_port_t i2cPort;
    int i2cAddress;

    int16_t temp;
    int16_t humidity;
    bool isMeasured;

    unsigned short crc16(unsigned char *ptr, unsigned char len);
    bool wakeUp();
    bool setReadRegs();
    bool readRegs();

};

#endif
#endif