#include <CompilationOpts.h>

#ifdef USE_IVAN_SUPERVISOR

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "IvanSupervisor.h"

void IvanSupervisorLedIndicators::init(UEventLoop *eventLoop, ServiceCommands *cmd, LedIndicatorService *ledIndicator, Logger *logger)
{
    this->ledIndicator = ledIndicator;

    isWithLoad = false;
    isWithWifi = false;
    isWithAutoShutdown = true;
    lastEvent = LI_ON;

    onNoLoad = -1;
    onWithLoad = -1;
    onWifiOnNoLoad = -1;
    onWifiOnWithLoad = -1;
    loadRunning = -1;
    loadRunningWifiOn = -1;
    idle = -1;
    idleNoShutdown = -1;
    idleWifiOn = -1;
    idleWifiOnNoShutdown = -1;
    idleShuttingDown = -1;
    buttonClick = -1;
    buttonLongClick = -1;

    // Commands

    auto registerAnimCmd = [this, cmd](const char *name, const char *help, int *id) {
        cmd->registerStringData(
            ServiceCommands::StringDataBuilder(name, true)
            .cmd(name)
            .help(help)
            .setFn([this, id](const String &val, bool isLoading, String *msg) {
                String err;
                int newId = this->ledIndicator->defineAnimation(val.c_str(), &err);
                if (newId == -1) {
                    *msg = "Error defining led indicator animation to " + val + ": " + err;
                } else {
                    *id = newId;
                    *msg = "Animation id " + String(*id);                    
                }
                return true;
            })
            .getFn([this, id](String *val) {
                bool rc = this->ledIndicator->getAnimationDef(*id, val);
                if (!rc) {
                    *val = "Animation not defined, id = " + *id;
                }
            })
        );
    };

    registerAnimCmd("ledIndicator.onNoLoad",
        "--> Led indicator animation when turned on, with no load expected; see service ledIndicator",
        &onNoLoad);
    registerAnimCmd("ledIndicator.onWithLoad",
        "--> Led indicator animation when turned on and the load will run; see service ledIndicator",
        &onWithLoad);
    registerAnimCmd("ledIndicator.onWifiOnNoLoad",
        "--> Led indicator animation when Wifi is turned on, with no load expected; see service ledIndicator",
        &onWifiOnNoLoad);
    registerAnimCmd("ledIndicator.onWifiOnWithLoad",
        "--> Led indicator animation when Wifi is turned on and the load will run; see service ledIndicator",
        &onWifiOnWithLoad);
    registerAnimCmd("ledIndicator.loadRunning",
        "--> Led indicator animation while the load is running, wifi is off; see service ledIndicator",
        &loadRunning);
    registerAnimCmd("ledIndicator.loadRunningWifiOn",
        "--> Led indicator animation while the load is running, wifi is on; see service ledIndicator",
        &loadRunningWifiOn);
    registerAnimCmd("ledIndicator.idle",
        "--> Led indicator animation while idle, no wifi, auto shutdown is set; see service ledIndicator",
        &idle);
    registerAnimCmd("ledIndicator.idleNoShutdown",
        "--> Led indicator animation while idle, no wifi, without auto shutdown; see service ledIndicator",
        &idleNoShutdown);
    registerAnimCmd("ledIndicator.idleWifiOn",
        "--> Led indicator animation while idle, with wifi, auto shutdown is set; see service ledIndicator",
        &idleWifiOn);
    registerAnimCmd("ledIndicator.idleWifiOnNoShutdown",
        "--> Led indicator animation while idle, with wifi, without auto shutdown; see service ledIndicator",
        &idleWifiOnNoShutdown);
    registerAnimCmd("ledIndicator.idleShuttingDown",
        "--> Led indicator animation about to shut down; see service ledIndicator",
        &idleShuttingDown);
    registerAnimCmd("ledIndicator.buttonClick",
        "--> Led indicator animation for button click; see service ledIndicator",
        &buttonClick);
    registerAnimCmd("ledIndicator.buttonLongClick",
        "--> Led indicator animation for button long click; see service ledIndicator",
        &buttonLongClick);

}

void IvanSupervisorLedIndicators::event(LedIndicatorEvent event)
{
    switch (event) {
        case LI_ON:
            if (isWithWifi) {
                if (isWithLoad) {
                    ledIndicator->setAnimation(0, this->onWifiOnWithLoad, 0);
                } else {
                    ledIndicator->setAnimation(0, this->onWifiOnNoLoad, 0);
                }
            } else {
                if (isWithLoad) {
                    ledIndicator->setAnimation(0, this->onWithLoad, 0);
                } else {
                    ledIndicator->setAnimation(0, this->onNoLoad, 0);
                }
            }
            break;
        case LI_LOAD_RUNNING:
            if (isWithWifi) {
                ledIndicator->setAnimation(0, this->loadRunningWifiOn, 0);
            } else {
                ledIndicator->setAnimation(0, this->loadRunning, 0);
            }
            break;
        case LI_LOAD_DONE:
            if (isWithWifi) {
                if (isWithAutoShutdown) {
                    ledIndicator->setAnimation(0, this->idleWifiOn, 0);
                } else {
                    ledIndicator->setAnimation(0, this->idleWifiOnNoShutdown, 0);
                }
            } else {
                if (isWithAutoShutdown) {
                    ledIndicator->setAnimation(0, this->idle, 0);
                } else {
                    ledIndicator->setAnimation(0, this->idleNoShutdown, 0);
                }
            }
            break;
        case LI_SHUTTING_DOWN:
            ledIndicator->setAnimation(0, this->idleShuttingDown, 0);
            break;
        case LI_BUTTON_CLICK:
            ledIndicator->setAnimation(1, this->buttonClick, 1);
            break;
        case LI_BUTTON_LONG_CLICK:
            ledIndicator->setAnimation(1, this->buttonLongClick, 1);
            break;
        default:
            break;
    }
    lastEvent = event;
}

void IvanSupervisorLedIndicators::setWithLoad(bool enabled)
{
    isWithLoad = enabled;
    event(lastEvent);
}

void IvanSupervisorLedIndicators::setWithWifi(bool enabled)
{
    isWithWifi = enabled;
    event(lastEvent);
}

void IvanSupervisorLedIndicators::setWithAutoShutdown(bool enabled)
{
    isWithAutoShutdown = enabled;
    event(lastEvent);
}

#endif
