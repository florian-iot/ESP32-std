#include <CompilationOpts.h>

#ifdef USE_COMM433
#ifndef INCL_COMM433_H
#define INCL_COMM433_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <RadioHead.h>
#include <RH_ASK.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"

class Comm433Service {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr);

    // event sent upon data receipt: "comm433:rxData", with the following structure:
    struct ReceivedMsg {
        uint16_t len;
        char *data;
    };

private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;

    int rxPin; // set to -1 to disable RX
    int txPin; // set to -1 to disable TX
    RH_ASK *driver;

    UEventLoopTimer readerTimer;
    UEventLoopTimer senderTimer;
    uint32_t rxEventType;
    bool isAutosend;
    int sendPeriod;
    String sendData; // data to send each period
    int sentCnt;

};

#endif
#endif