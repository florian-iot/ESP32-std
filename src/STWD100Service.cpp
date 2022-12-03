#include <CompilationOpts.h>

#ifdef USE_STWD100

#include <HardwareSerial.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "STWD100Service.h"
#include "LogMgr.h"
#include "Util.h"

void STWD100Service::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("stwd100");

    ServiceCommands *cmd = commandMgr->getServiceCommands("stwd100");
    initCommands(cmd);

    // defaults
    pinWdIn = -1;
    isKeepAlwaysAlive = true;
    isWatchdogTriggered = false;
    isAutoTriggerNext = false;
    interval = 500;
    loopCount = 0;

    // load config
    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    // init data structures

    // init hardware
    if (pinWdIn != -1) {
        digitalWrite(pinWdIn, LOW);
        pinMode(pinWdIn, OUTPUT);
    }

    eventLoop->registerTimer(&timer);
    timer.setInterval([this](UEventLoopTimer *timer) {
        runWatchdogLoop();
    }, interval);

    Util::runAsThread("WatchdogNotifier", [this] {
        unsigned long lastLoopTime = millis();
        bool isPinHigh = false;
        unsigned lastLoopCount = loopCount;
        while (true) {
            delay(interval / 2);
            // check that the last execution of the event loop was no more than 3 seconds ago

            int curLoopCount = loopCount; // loopCount is volatile
            if (curLoopCount != lastLoopCount) {
                lastLoopTime = millis();
                lastLoopCount = curLoopCount;
            }

            if ((millis() - lastLoopTime < 3000) || isKeepAlwaysAlive) { // as long as we're within 3 seconds, do notify the watchdog
                if (pinWdIn != -1) {
                    isPinHigh = !isPinHigh;
                    digitalWrite(pinWdIn, isPinHigh ? HIGH : LOW);
                }
            }
        }
    });

}

void STWD100Service::initCommands(ServiceCommands *cmd)
{
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("pinWdIn", true)
        .cmd("pinWdIn")
        .help("--> Set watchdog IN pin, set to -1 for no watchdog")
        .vMin(-1)
        .vMax(99)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            if (isLoading) {
                pinWdIn = val;
            } else {
                if (pinWdIn != -1) {
                    digitalWrite(pinWdIn, LOW);
                    pinMode(pinWdIn, INPUT);
                }
                pinWdIn = val;
                if (pinWdIn != -1) {
                    digitalWrite(pinWdIn, LOW);
                    pinMode(pinWdIn, OUTPUT);
                }
            }
            *msg = "Watchdog IN pin set to "; *msg += pinWdIn;
            return true;
        })
        .getFn([this]() {
            return pinWdIn;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("alwaysAlive", true)
        .cmdOn("alwaysAlive")
        .helpOn("--> If set to true, the watchdog will always be notified, effectively disabling the watchdog triggering functionality")
        .ptr(&isKeepAlwaysAlive)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("interval", true)
        .cmd("interval")
        .help("--> Watchdog notification interval, in milliseconds")
        .vMin(1)
        .vMax(10000)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            interval = val;
            *msg = "Interval set to "; msg->concat(interval); msg->concat(" millis");
            return true;
        })
        .getFn([this]() {
            return interval;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("reset", true)
        .cmdOn("reset")
        .helpOn("--> Reset watchdog status")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (isWatchdogTriggered) {
                isWatchdogTriggered = false;
                *msg = "Watchdog status was reset";
            } else {
                *msg = "Watchdog has not been triggered, nothing to reset";
            }
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("trigger", true)
        .cmdOn("trigger")
        .helpOn("--> Trigger the watchdog")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (isWatchdogTriggered) {
                *msg = "Watchdog is already in triggered status";
            } else {
                isAutoTriggerNext = true;
                *msg = "Set to trigger on next iteration";
            }
            return true;
        })
    );

    cmd->onAfterStatus([this](String *msg) {
        msg->concat("    isWatchdogTriggered: ");
        msg->concat(isWatchdogTriggered ? "true" : "false");
        msg->concat("\n    triggerCause: ");
        msg->concat(triggerCause);
        msg->concat("\n");
    });
}

int STWD100Service::registerVerification(std::function<bool (String *cause)> verifyFn)
{
    return verifyList.add(verifyFn);
}

void STWD100Service::unregisterVerification(int handle)
{
    verifyList.remove(handle);
}

int STWD100Service::registerNotification(std::function<void (const String *cause)> notifyFn)
{
    return notifyList.add(notifyFn);
}

void STWD100Service::unregisterNotification(int handle)
{
    notifyList.remove(handle);
}


void STWD100Service::runWatchdogLoop()
{
    if (isWatchdogTriggered) {
        return;
    }
    bool isOk = true;
    String cause;

    if (isAutoTriggerNext) {
        isOk = false;
        cause = "STWD100: autoTrigger";
        isAutoTriggerNext = false;
    }
    for (auto i = 0; isOk && i < verifyList.size(); i++) {
        if (!verifyList.get(i)(&cause)) {
            isOk = false;
        }
    }

    if (isOk) {
        loopCount++;
    } else {
        triggerCause = cause;
        logger->error("Watchdog triggered, cause: {}", triggerCause.c_str());
        for (int i = 0; i != notifyList.size(); i++) {
            notifyList.get(i)(&cause);
        }
        isWatchdogTriggered = true;
        // and we don't pulse pinWdIn, so the watchdog will reboot the MCU (unless isKeepAlwaysAlive)
    }
}

#endif
