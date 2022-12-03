#include <CompilationOpts.h>

#if defined(USE_GPSDO) && defined(GPSDO_GEN)
#ifndef INCL_GPSDO_GEN_H
#define INCL_GPSDO_GEN_H

#include <HardwareSerial.h>
#include <driver/ledc.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "SystemService.h"
#include "MqttService.h"

class GpsdoGen {
public:
    void init(const char *serviceName, UEventLoop *eventLoop, SystemService *systemService,
        MqttService *mqtt, ServiceCommands *cmd, Logger *logger);
    // after init config is loaded
    void postInit();
private:
    UEventLoop *eventLoop;
    Logger *logger;
    MqttService *mqtt;

    bool isGenEnabled;
    SysPin genPpsPin;
    SysPin genPulsePin;
    int genPulseFreq;
    int genPulseDiv;

    ledc_timer_config_t ledcTimerPps;
    ledc_channel_config_t ledcChannelPps;
    ledc_timer_config_t ledcTimerPulse;
    ledc_channel_config_t ledcChannelPulse;

    void initCommands(ServiceCommands *cmd);

    void enableGen();
    void disableGen();
    bool setGenPulseFreq(int freq);
};

#endif
#endif