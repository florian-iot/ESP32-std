#include <CompilationOpts.h>

#ifdef USE_REBOOT_DETECTOR

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "RebootDetectorService.h"
#include "LogMgr.h"

RebootDetectorService::RebootDetectorService()
: fileName("/rebootDetector.txt")
{
    quickRebootsCount = 0;
}

void RebootDetectorService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr,  SystemService *systemService, FS *fs)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("rebootDetector");
    this->systemService = systemService;
    this->fs = fs;
    quickRebootsCount = 0;
    involuntaryShutdown = false;
    isEnabled = false;

    ServiceCommands *cmd = commandMgr->getServiceCommands("rebootDetector");
    initCommands(cmd);

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

    if (fs && isEnabled) {
        fs::File fd = fs->open(fileName, "r+");
        if (!fd) {
            fd = fs->open(fileName, "w+");
        }
        String line = fd.readString();
        if (line.isEmpty()) {
            fd.seek(0);
            fd.print("-1"); // to consider involuntary shutdown if not reset to 0 upon shutdown
            fd.flush();
        } else {
            quickRebootsCount = line.toInt();
            if (quickRebootsCount == -1) {
                quickRebootsCount = 0;
                involuntaryShutdown = true;
            }
        }
        fd.close();

        fd = fs->open(fileName, "w+"); // to truncate
        fd.print(quickRebootsCount + 1);
        fd.close();

        systemService->onShutdown([this]() {
            fs::File fd = this->fs->open(fileName, "w+");
            fd.print("0"); // this is a voluntary shutdown
            fd.close();
        });

        // if we manage to survive for 10 seconds, we'll reset the stored quickRebootCounts, we consider booting is OK
        timer.init(eventLoop, [this](UEventLoopTimer *timer) {
            fs::File fd = this->fs->open(fileName, "w+");
            fd.print("-1"); // reset stored counter, but if not set to 0 upon shutdown, will be considered involontary shutdown
            fd.close();

            // if we had a reboot level > 0, and we managed to reset it here, we
            // must reboot after 15 minutes - unless we receive a reset

            if (getRebootLevel() != Level::APPLICATION) {
                timer->setTimeout([this](UEventLoopTimer *timer) {
                    logger->info("Rebooting after 15 minutes at reboot level {}", getRebootLevelStr());
                    Serial.printf("Rebooting after 15 minutes at reboot level %s", getRebootLevelStr());
                    this->systemService->setMustReboot();
                }, 15 * 60 * 1000);
            }

        });
        timer.setTimeout(10000);
    }
}

bool RebootDetectorService::isActive()
{
    return isEnabled;
}

// May be called before init(), in which case will always return 0
RebootDetectorService::Level RebootDetectorService::getRebootLevel()
{
    if (quickRebootsCount < 4) {
        return APPLICATION;
    } else if (quickRebootsCount < 6) {
        return INFRASTRUCTURE;
    } else {
        return MINIMUM;
    }
}

const char *RebootDetectorService::getRebootLevelStr()
{
    if (quickRebootsCount < 4) {
        return "APPLICATION";
    } else if (quickRebootsCount < 6) {
        return "INFRASTRUCTURE";
    } else {
        return "MINIMUM";
    }
}

void RebootDetectorService::resetRebootLevel()
{
    if (isEnabled && quickRebootsCount > 0) {
        quickRebootsCount = 0;
        if (fs) {
            fs::File fd = fs->open(fileName, "w+");
            fd.print(0);
            fd.close();
        }
        timer.cancelTimeout();
    }
}

bool RebootDetectorService::isInvoluntaryShutdown()
{
    return involuntaryShutdown;
}

void RebootDetectorService::initCommands(ServiceCommands *cmd)
{
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("enable", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .helpOn("--> Enable reboot detector, requires reboot")
        .helpOff("--> Disable reboot detector, requires reboot")
        .ptr(&isEnabled)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("quickRebootsCount", true)
        .cmd("quickRebootsCount")
        .help("--> Number of quick reboots (resets to 0 after 10 seconds, increases if last reboot was less than 10 second after boot")
        .isPersistent(false)
        .getFn([this] { return quickRebootsCount; })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("rebootLevel", true)
        .cmd("rebootLevel")
        .help("--> Level 0 is normal, 1 when a few quick reboots have happened, 2 when more quick reboots have happened.")
        .isPersistent(false)
        .getFn([this] { return getRebootLevel(); })
    );
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("reset", true)
        .cmdOn("reset")
        .helpOn("--> Reset quickRebootsCount and unarm triggering of reboot after 15 minutes")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            resetRebootLevel();
            return true;
        })
    );
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("involuntaryShutdown", true)
        .cmd("involuntaryShutdown")
        .help("--> True if last shutdown was involuntary")
        .isPersistent(false)
        .getFn([this] { return involuntaryShutdown; })
    );
}

#endif
