#include <CompilationOpts.h>

#ifdef USE_LM75A_TEMP
#ifndef INCL_LM75A_TEMP_H
#define INCL_LM75A_TEMP_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "SystemService.h"


class LM75A {
    int i2cAddress;
    i2c_port_t i2cPort;
    uint8_t regConfig;
    bool isInitialized;

    enum {
        REG_CONFIG = 1,
        REG_TEMP = 0,
        REG_THYST = 2,
        REG_TOS = 3,
        REG_PROD_ID = 7
    };

    bool checkConnect();
    bool readConfig();
    bool readRegister(uint8_t reg, uint8_t *values, int length);
    bool writeRegister(uint8_t reg, uint8_t *values, int length);
public:
    bool init(int i2cAddress, i2c_port_t i2cPort);
    void shutdown();
    void wakeup();
    bool isShutdown();
    // 999999 for invalid
    int getTempMilliC();
    // 999999 for invalid
    int getHystTempMilliC();
    void setHystTempMilliC(int tempMilliC);
    // 999999 for invalid
    int getOSTripTempMilliC();
    void setOsTripTemperature(int tempMilliC);
    // 0 for low, 1 for high
    int getOsPolarity();
    // 0 for low, 1 for high
    void setOsPolarity(int polarity);
    // 1, 2, 4 or 6
    int getFaultQueueValue();
    // 1, 2, 4 or 6
    void setFaultQueueValue(int value);
    // 0 for comparator, 1 for interrupt
    int getDeviceMode();
    // 0 for comparator, 1 for interrupt
    void setDeviceMode(int deviceMode);
    bool checkProdId();
    int readProdId();
    bool isConnected();

    bool readReg8(uint8_t reg, uint8_t *value);
    bool readReg16(uint8_t reg, uint16_t *value);
    bool writeReg8(uint8_t reg, uint8_t value);
    bool writeReg16(uint8_t reg, uint16_t value);
};

inline bool LM75A::checkConnect()
{
    if (!isInitialized) {
        isInitialized = /* checkProdId() && */ isConnected() && readConfig();
    }
    return isInitialized;
}


class LM75A_TempService {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, SystemService *systemService, LogMgr *logMgr);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;

    bool isEnabled;
    int i2cAddress;
    i2c_port_t i2cPort;
    LM75A lm75a;

    bool isLogging;
    int tempOffsetMilliC;

    UEventLoopTimer loggingTimer;
public:
    // Returns 999999 if impossible to read temp
    int readTempMilliC();
};

#endif
#endif
