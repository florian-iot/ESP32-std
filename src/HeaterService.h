#include <CompilationOpts.h>

#ifdef USE_HEATER
#ifndef INCL_HEATER_H
#define INCL_HEATER_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "SystemService.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include <driver/ledc.h>
#include <driver/pcnt.h>

class HeaterService {
public:
    void init(UEventLoop *eventLoop, SystemService *systemService, CommandMgr *commandMgr, LogMgr *logMgr);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    SystemService *systemService;
    Logger *logger;

    void initCommands(ServiceCommands *cmd);

    SysPin heaterTempAdcPin;
    SysPin airTempAdcPin;

    SysPin heaterPwmPin;
    SysPin fanRpmSensorPin;

    ledc_timer_t pwmTimer;
    ledc_channel_t heaterPwmChannel;
    ledc_channel_config_t heaterPwmChannelConfig;
    bool heaterPwmIsInverted;
    SysPin fanPwmPin;
    ledc_channel_t fanPwmChannel;
    ledc_channel_config_t fanPwmChannelConfig;
    bool fanPwmIsInverted;

    pcnt_unit_t fanRpmCounterUnit;
    UEventLoopTimer fanRpmCounterTimer;
    int64_t fanRpmLastTime;
    int fanRpm;

    int heater; // scale 0..1000
    int fan; // scale 0..1000

    adc1_channel_t heaterAdcChannel;
    esp_adc_cal_characteristics_t heaterAdcCharacteristics;
    adc_atten_t heaterAdcAtten;
    int heaterAdcOversampling;
    adc1_channel_t airAdcChannel;
    esp_adc_cal_characteristics_t airAdcCharacteristics;
    adc_atten_t airAdcAtten;
    int airAdcOversampling;

    int readAdc(adc1_channel_t channel, esp_adc_cal_characteristics_t *characteristics, int samples);

    void setHeaterDuty(int duty /* 0..1000 */);
    void setFanDuty(int duty /* 0..1000 */);

};

#endif
#endif