#include <CompilationOpts.h>

#ifdef USE_STWD100
#ifndef INCL_STWD100_
#define INCL_STWD100_H

#include <HardwareSerial.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "Util.h"

class STWD100Service {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;

    void initCommands(ServiceCommands *cmd);

    volatile int pinWdIn;
    volatile int interval;
    volatile unsigned loopCount;

    UEventLoopTimer timer;
    bool isKeepAlwaysAlive;
    bool isWatchdogTriggered;
    bool isAutoTriggerNext;
    String triggerCause;

    CallbackList<bool (String *cause)> verifyList;
    CallbackList<void (const String *cause)> notifyList;

    // std::vector<std::function<bool (String *cause)>> verifyList;
    // std::vector<std::function<void (const String *cause)>> notifyList;

    void runWatchdogLoop();


public:
    int registerVerification(std::function<bool (String *cause)> verifFn);
    void unregisterVerification(int handle);

    int registerNotification(std::function<void (const String *cause)> notifFn);
    void unregisterNotification(int handle);
};

#endif
#endif