#include <CompilationOpts.h>

#ifdef USE_GPSDO

#include <HardwareSerial.h>
#include <driver/ledc.h>
#include <soc/ledc_reg.h>
#include <soc/ledc_struct.h>
#include <freertos/FreeRTOS.h>

#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "SystemService.h"
#include "GpsdoDac.h"

void GpsdoDac::init(const char *serviceName, UEventLoop *eventLoop, SystemService *systemService,
    ServiceCommands *cmd, Logger *logger)
{
    this->eventLoop = eventLoop;
    this->logger = logger;
    isEnabled = false;
    isEnabledOnLoad = false;
    
    dacPin = systemService->registerSysPin(serviceName, "dac.pin");
    dacValue = DAC_MAX_VAL / 2;

    initCommands(cmd);
}

void GpsdoDac::postInit()
{
    if (dacPin != -1) {
        pinMode(dacPin.getPin(), OUTPUT);
        digitalWrite(dacPin, HIGH);
        if (isEnabledOnLoad) {
            enable();
        }
    }
}

void GpsdoDac::enable()
{
    if (dacPin.getPin() == -1) {
        return;
    }

    ledcTimerDac = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        { .duty_resolution = (ledc_timer_bit_t)DAC_RESOLUTION_BITS },
        .timer_num = LEDC_TIMER_2,
        .freq_hz = DAC_FREQ
    };
    esp_err_t rc = ledc_timer_config(&ledcTimerDac);
    if (rc == ESP_OK) {
        // // set divider manually, so that we don't count on frequency calculations (we don't want jitter in frequency)
        // rc = ledc_timer_set(ledcTimerDac.speed_mode, ledcTimerDac.timer_num,
        //     DAC_TIMER_DIVIDER, DAC_RESOLUTION_BITS, LEDC_APB_CLK);
    }
    if (rc != ESP_OK) {
        logger->error("Error configuring pwm timer for dac");
        return;
    }

    ledcChannelDac = {
        .gpio_num = dacPin,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_2,
        .duty = (DAC_MAX_VAL - dacValue - (dacValue == 0 ? 1 : 0)),
        .hpoint = 0
    };
    rc = ledc_channel_config(&ledcChannelDac);
    if (rc != ESP_OK) {
        logger->error("Error configuring pwm channel for dac");
        return;
    }

    setValue(dacValue);

    isEnabled = true;
}

void GpsdoDac::disable()
{
    if (!isEnabled) {
        return;
    }
    ledc_stop(ledcChannelDac.speed_mode, ledcChannelDac.channel, LOW);
    isEnabled = false;
//    ledc_timer_pause(ledcChannelDac.speed_mode, ledcChannelDac.channel);
//    gpio_set_level((gpio_num_t)ledcChannelDac.gpio_num, LOW);
}

// https://github.com/espressif/esp-idf/blob/release/v3.3/components/driver/ledc.c

bool GpsdoDac::setDuty(uint32_t dutyFrac)
{
    // ledc_duty_config(speed_mode,
    //                  channel,         //uint32_t chan_num,
    //                  hpoint,          //uint32_t hpoint_val,
    //                  duty << 4,       //uint32_t duty_val,the least 4 bits are decimal part
    //                  1,               //uint32_t increase,
    //                  1,               //uint32_t duty_num,
    //                  1,               //uint32_t duty_cycle,
    //                  0                //uint32_t duty_scale
    //                  );
    if (dutyFrac >= 0) {
        LEDC.channel_group[ledcChannelDac.speed_mode].channel[ledcChannelDac.channel].duty.duty = dutyFrac;
    }
    LEDC.channel_group[ledcChannelDac.speed_mode].channel[ledcChannelDac.channel].conf1.val =
            ((1 /* duty_direction */ & LEDC_DUTY_INC_HSCH0_V) << LEDC_DUTY_INC_HSCH0_S) |
            ((1 /* duty_num */ & LEDC_DUTY_NUM_HSCH0_V) << LEDC_DUTY_NUM_HSCH0_S) |
            ((1 /* duty_cycle */ & LEDC_DUTY_CYCLE_HSCH0_V) << LEDC_DUTY_CYCLE_HSCH0_S) |
            ((0 /* duty_scale */ & LEDC_DUTY_SCALE_HSCH0_V) << LEDC_DUTY_SCALE_HSCH0_S);

    esp_err_t rc = ledc_update_duty(ledcChannelDac.speed_mode, ledcChannelDac.channel);
    if (rc == ESP_OK) {
        dacValue = dutyFrac;
    }
    return (rc == ESP_OK);
}

void GpsdoDac::initCommands(ServiceCommands *cmd)
{
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&dacPin)
        .help("--> Output pin used for PWM DAC")
    );

    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("dac.enabled", true)
        .cmdOn("dac.enable")
        .cmdOff("dac.disable")
        .helpOn("--> Enable DAC output")
        .helpOff("--> Disable DAC output (output remains low)")
        .ptr(&isEnabled)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (isLoading) {
                isEnabledOnLoad = val;
                return true;
            }
            if (val) {
                enable();
            } else {
                disable();
            }
            if (val) {
                *msg = (isEnabled ? "DAC is enabled" : "DAC was not enabled, please see logs");
            } else {
                *msg = (!isEnabled ? "DAC is disabled" : "DAC is still enabled, please see logs");
            }
            return true;
        })
    );

    cmd->registerIntData(ServiceCommands::IntDataBuilder("dac.value", true)
        .cmd("dac.value")
        .help("--> Set DAC output value, maximum is " DAC_MAX_VAL_STR)
        .vMin(0)
        .vMax(DAC_MAX_VAL)
        .setFn([this](int val, bool isLoading, String *msg) {
            if (isLoading) {
                dacValue = val;
                return true;
            }
            bool rc = setValue(val);
            *msg = (rc ? "Value set" : "Error setting value");
            return true;
        })
        .getFn([this]() {
            return getValue();
        })
    );

    cmd->registerIntData(ServiceCommands::IntDataBuilder("dac.inc", true)
        .cmd("dac.inc")
        .help("--> Increase (or decrease, if negative) the current DAC output value, maximum is " DAC_MAX_VAL_STR)
        .isPersistent(false)
        .setFn([this](int val, bool isLoading, String *msg) {
            if (val > 0) {
                if ((int32_t)getValue() + val <= 0 || (int32_t)getValue() + val > DAC_MAX_VAL) {
                    bool rc = setValue(DAC_MAX_VAL);
                    if (rc) {
                        *msg = "DAC output value set to maximum " DAC_MAX_VAL_STR;
                    } else {
                        *msg = "Error setting DAC value";
                    }
                } else {
                    bool rc = setValue(getValue() + (uint32_t)val);
                    if (rc) {
                        *msg = "DAC output value set to "; msg->concat(dacValue);
                    } else {
                        *msg = "Error setting DAC value";
                    }
                }
            } else {
                if ((int32_t)dacValue + val <= 0 || (int)dacValue + val > DAC_MAX_VAL) {
                    bool rc = setValue(0);
                    if (rc) {
                        *msg = "DAC output value set to 0";
                    } else {
                        *msg = "Error setting DAC value";
                    }
                } else {
                    bool rc = setValue((int32_t)getValue() + (int32_t)val);
                    if (rc) {
                        *msg = "DAC output value set to "; msg->concat(dacValue);
                    } else {
                        *msg = "Error setting DAC value";
                    }
                }
            }
            return true;
        })
    );
}

uint32_t GpsdoDac::getValue()
{
    return dacValue;
}

bool GpsdoDac::setValue(uint32_t val)
{
    if (isEnabled) {
        bool rc = setDuty(DAC_MAX_VAL - val - (val == 0 ? 1 : 0));
        if (rc) {
            dacValue = val;
        }
        return rc;
    } else {
        dacValue = val;
        return true;
    }
}

uint32_t GpsdoDac::getMaxValue()
{
    return DAC_MAX_VAL;
}


#endif