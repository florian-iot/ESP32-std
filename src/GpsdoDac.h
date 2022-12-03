#include <CompilationOpts.h>

#ifdef USE_GPSDO
#ifndef INCL_GPSDO_DAC_H
#define INCL_GPSDO_DAC_H

#include <HardwareSerial.h>
#include <driver/ledc.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "SystemService.h"
#include "GpsdoDac.h"

/**
 * Freq vs. DAC value - 1st OXCO
 * Every 10000, starting from 10000, up to 120000
 * 212
 * 210.5
 * 208.5
 * 206
 * 203
 * 200
 * 196.5
 * 193.5
 * 190.5
 * 188.5
 * 185.5
 * 184.5
 */


// Divide 40 MHz by 10000 -> 4 kHz. Divider is fixed-point with 8 fraction bits,
// we'll set the fraction bits to 0, so there's no jitter
#define DAC_TIMER_DIVIDER (10000 << 8)
#define DAC_FREQ 4000
// Each PWM cycle has 10000 clocks, and that corresponds to 4 kHz.
// Resolution of duty cycle must go to less than 10000 clocks and must be
// a power of two, so it is 8192 == 2^13
#define DAC_RESOLUTION_BITS 13
// User-visible maximum value for duty will include 4 lowest bits that ledc uses
// for dithering.
#define DAC_MAX_VAL ((1 << DAC_RESOLUTION_BITS) << 4)
#define DAC_MAX_VAL_STR "131072"

class GpsdoDac {
public:
    void init(const char *serviceName, UEventLoop *eventLoop, SystemService *systemService, ServiceCommands *cmd, Logger *logger);
    // after init config is loaded
    void postInit();

    void start();
    uint32_t getValue();
    uint32_t getMaxValue();
    bool setValue(uint32_t val);
private:
    UEventLoop *eventLoop;
    Logger *logger;

    bool isEnabled;
    bool isEnabledOnLoad;
    SysPin dacPin;

    ledc_timer_config_t ledcTimerDac;
    ledc_channel_config_t ledcChannelDac;
    uint32_t dacValue;

    void initCommands(ServiceCommands *cmd);

    void enable();
    void disable();
    bool setDuty(uint32_t duty);
};

#endif
#endif