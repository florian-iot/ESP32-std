#include <CompilationOpts.h>

#ifdef USE_HEARTBEAT
#ifndef INCL_HEARTBEAT_H
#define INCL_HEARTBEAT_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"

class HeartbeatService {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;

    Dfa dfa;

    Dfa::Input ENABLE = dfa.nextInput("ENABLE");
    Dfa::Input DISABLE = dfa.nextInput("DISABLE");

    Dfa::State HB_STARTUP = dfa.nextState("HB_STARTUP");
    Dfa::State HB_DO_ENABLE = dfa.nextState("HB_DO_ENABLE");
    Dfa::State HB_ENABLED = dfa.nextState("HB_ENABLED");
    Dfa::State HB_DISABLED = dfa.nextState("HB_DISABLED");

    void initCommands(ServiceCommands *cmd);
    void initDfa();

    bool isEnabled;
    bool isSerialDot;
    int ledPin;
    int period;
    bool isInverted;

    long cnt;
    int index;
    int durations[4];
};

#endif
#endif