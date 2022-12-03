#include <CompilationOpts.h>

#ifdef USE_LED_SPHERE
#ifndef INCL_LED_SPHERE_H
#define INCL_LED_SPHERE_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "ButtonService.h"
#include "BeeperService.h"
#include "LedService.h"
#include "OTA.h"
#include "SystemService.h"

/*

8 LEDs
14 LEDs
18 LEDs
20 LEDs
24 LEDs
26 LEDs
24
20
18
14
8
*/


class LedSphereService {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr,
        SystemService *systemService, ButtonService *buttonService, BeeperService *beeperService,
        LedService *ledService, OTA *ota);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;
    ButtonService *buttonService;
    BeeperService *beeperService;
    LedService *ledService;

    DynamicJsonBuffer clickCommandsBuffer;
    JsonArray *startupEnterCommands; // array of strings
    JsonArray *startupLeaveCommands; // array of strings
    JsonArray *clickCommands; // array of arrays of strings
    JsonArray *batteryLowEnterCommands; // array of strings
    JsonArray *batteryLowLeaveCommands; // array of strings
    JsonArray *usbPowerEnterCommands; // array of strings
    JsonArray *usbPowerLeaveCommands; // array of strings

    SysPin voltageSensePin;
    SysPin chargingSensePin;
    int voltageDividerEnumerator;
    int voltageDividerDenominator;
    int batteryLowVoltage; // in millivolt
    int batteryLowVoltageHyst; // up from batteryLowVoltage to be considered out of low voltage status
    int usbPowerVoltage; // at or above this voltage, we have USB power
    int usbPowerVoltageHyst; // hysteresis for usbPowerVoltage (always non-negative)

    int clickIndex;
    uint32_t avgBatteryVoltage;
    UEventLoopTimer batteryVoltageTimer;
    bool isBatteryLow;
    bool isUsbPower;
    UEventLoopTimer startSequenceTimer;

    Dfa dfa;

    Dfa::State LS_STARTUP = dfa.nextState("LS_STARTUP");
    Dfa::State LS_START_SEQUENCE = dfa.nextState("LS_START_SEQUENCE");
    Dfa::State LS_START_ACTIVE = dfa.nextState("LS_START_ACTIVE");
    Dfa::State LS_ACTIVE = dfa.nextState("LS_ACTIVE");
    Dfa::State LS_BATTERY_LOW = dfa.nextState("LS_BATTERY_LOW");
    Dfa::State LS_ON_USB_STANDBY = dfa.nextState("LS_ON_USB_STANDBY");

    Dfa::Input START = dfa.nextInput("START");
    Dfa::Input CLICK = dfa.nextInput("CLICK");
    Dfa::Input BATTERY_LOW = dfa.nextInput("BATTERY_LOW");
    Dfa::Input BATTERY_LOW_TERMINATED = dfa.nextInput("BATTERY_LOW_TERMINATED");
    Dfa::Input USB_POWER = dfa.nextInput("USB_POWER");
    Dfa::Input USB_POWER_TERMINATED = dfa.nextInput("USB_POWER_TERMINATED");
    Dfa::Input QUIT_ON_USB_STANDBY = dfa.nextInput("QUIT_ON_USB_STANDBY");

    int dfaStartSeq;
    int dfaStartSparkDensityPct;

    void initCommands(ServiceCommands *cmd);
    void initDfa();

    void voltageCheck();
    void chargingCheck();

};

#endif
#endif