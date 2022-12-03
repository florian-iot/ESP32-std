#include <CompilationOpts.h>

#ifdef USE_GPSDO
#ifndef INCL_GPSDO_H
#define INCL_GPSDO_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <driver/pcnt.h>
#include <driver/ledc.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "SystemService.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "MqttService.h"
#include "GpsdoDac.h"
#include "GpsdoGen.h"

#define PCNT_UNIT_LONG_GATE PCNT_UNIT_0
#define PCNT_UNIT_PPS_RAISING PCNT_UNIT_1
#define PCNT_UNIT_PPS_FALLING PCNT_UNIT_2

class GpsdoFreqCounterService {
public:
    void init(UEventLoop *eventLoop, SystemService *systemService,
#ifdef USE_MQTT
    MqttService *mqtt,
#endif
    CommandMgr *commandMgr, LogMgr *logMgr);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;
#ifdef USE_MQTT
    MqttService *mqtt;
#endif

    UEventLoop *ppsEventLoop;

    void initCommands(ServiceCommands *cmd);

    GpsdoGen gen;
    GpsdoDac dac;

    Dfa ppsDfa;
    Dfa::State FC_STARTUP = ppsDfa.nextState("FC_STARTUP");
    Dfa::Input FC_PPS = ppsDfa.nextInput("FC_PPS");
    Dfa::Input FC_PPS_SYNC_LOST = ppsDfa.nextInput("FC_PPS_SYNC_LOST");
    Dfa::Input FC_START = ppsDfa.nextInput("FC_START");
    Dfa::State FC_WAIT_BEFORE_FIRST_PPS = ppsDfa.nextState("FC_WAIT_BEFORE_FIRST_PPS");
    Dfa::State FC_WAIT_BEFORE_FIRST_PPS_PAST = ppsDfa.nextState("FC_WAIT_BEFORE_FIRST_PPS_PAST");
    Dfa::State FC_WAIT_FIRST_PPS = ppsDfa.nextState("FC_WAIT_FIRST_PPS");
    Dfa::State FC_WAIT_FIRST_PPS_PAST = ppsDfa.nextState("FC_WAIT_FIRST_PPS_PAST");
    Dfa::State FC_WAIT_NEXT_PPS = ppsDfa.nextState("FC_WAIT_NEXT_PPS");
    Dfa::State FC_WAIT_NEXT_PPS_PAST = ppsDfa.nextState("FC_WAIT_NEXT_PPS_PAST");
    Dfa::State FC_WAIT_LAST_PPS = ppsDfa.nextState("FC_WAIT_LAST_PPS");
    Dfa::State FC_WAIT_LAST_PPS_PAST = ppsDfa.nextState("FC_WAIT_LAST_PPS_PAST");
    Dfa::State FC_PPS_LOST = ppsDfa.nextState("FC_PPS_LOST");
// timeout set to go to _PAST states, after PPS encouterned
#define PAST_MILLIS 50

    void initPpsDfa();

    bool isEnabled;

    SysPin ppsPin;
    SysPin pulsePin;
    SysPin flipFlopDataPin;
    SysPin ctrl1Pin;
    SysPin ctrl2Pin;
    SysPin dacPin;
    int filterLength;
    int gateSeconds;
    bool isDisciplining;

    pcnt_config_t pcntLongGateConfig;
    pcnt_config_t pcntPpsRaisingConfig;
    pcnt_config_t pcntPpsFallingConfig;
    int gateSecondsCount;
    pcnt_isr_handle_t isr_handle;
    int64_t counter;
    int64_t ppsCounter;
    int64_t ppsCounterCumulative; // cumulative for gateSeconds

    bool isPpsPresent;
    int firstPpsCount;
    int64_t ppsTimestamp;
    UEventLoopTimer ppsTimeoutTimer;

    // disciplining-related
    
    // Dac slope: change in dac value needed for a 1 Hz change in freq at 0, 1/4 of dac max val, at 1/2, at 3/4, at max val
    // For the current OCXO, 3333 dac values for 1 Hz change in the middle of the scale, half that at extremities
    int32_t dacCalibratedSlope[5];
    // Expected max pps jitter in nanoseconds (typically 50)
    int8_t ppsJitter;

    // current value of dac slope, updated from the measured impact of dac value changes,
    // set at startup according to dacCalibratedSlope and dac value
    int32_t dacCurSlope;

    // duration in seconds since start of cumulating error
    uint32_t errorDuration;
    // measured cumulative error
    int32_t error;

    Dfa correctionDfa;
    Dfa::State C_STARTUP = correctionDfa.nextState("C_STARTUP");
    Dfa::State C_WARMUP = correctionDfa.nextState("C_WARMUP");
    Dfa::State C_COARSE_ADJ = correctionDfa.nextState("C_COARSE_ADJ");
    Dfa::State C_FINE_ADJ = correctionDfa.nextState("C_FINE_ADJ");




    void enable();
    void disable();
    void initCounter(pcnt_unit_t unit, pcnt_channel_t channel, pcnt_config_t *cfg, int pulsePin, int ctrlPin, bool isCtrlPinInverted);
    int64_t retrieveLongGateCounter();
    int64_t retrievePpsCounter();
    void publishStatus(const char *reason);

};

#endif
#endif