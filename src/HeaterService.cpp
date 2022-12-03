#include <CompilationOpts.h>

#ifdef USE_HEATER

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "HeaterService.h"
#include "LogMgr.h"
#include "SystemService.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include <driver/ledc.h>
#include <driver/pcnt.h>

static adc1_channel_t gpioToChannelMap[] = {
    ADC1_GPIO32_CHANNEL,
    ADC1_GPIO33_CHANNEL,
    ADC1_GPIO34_CHANNEL,
    ADC1_GPIO35_CHANNEL,
    ADC1_GPIO36_CHANNEL,
    ADC1_GPIO37_CHANNEL,
    ADC1_GPIO38_CHANNEL,
    ADC1_GPIO39_CHANNEL
};
#define gpioToChannel(gpio) \
    ((adc1_channel_t)((int)(gpio) >= 32 && (int)(gpio) <= 39 ? \
    gpioToChannelMap[(int)(gpio) - 32] : (adc1_channel_t)-1))

void HeaterService::init(UEventLoop *eventLoop, SystemService *systemService, CommandMgr *commandMgr, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->systemService = systemService;
    this->logger = logMgr->newLogger("heartbeat");

    // defaults
    heaterTempAdcPin = this->systemService->registerSysPin("heater", "heaterTempAdcPin");
    heaterTempAdcPin.setPin(34);
    heaterAdcAtten = ADC_ATTEN_6db;
    heaterAdcOversampling = 64;

    airTempAdcPin = this->systemService->registerSysPin("heater", "airTempAdcPin");
    airTempAdcPin.setPin(35);
    airAdcAtten = ADC_ATTEN_6db;
    airAdcOversampling = 64;

    heaterPwmPin = this->systemService->registerSysPin("heater", "heaterPwmPin");
    heaterPwmPin.setPin(23);
    pwmTimer = LEDC_TIMER_3;
    heaterPwmChannel = LEDC_CHANNEL_3;
    heaterPwmIsInverted = true;
    heater = 0;
    fanPwmPin = this->systemService->registerSysPin("heater", "fanPwmPin");
    fanPwmPin.setPin(22);
    fanPwmChannel = LEDC_CHANNEL_4;
    fanPwmIsInverted = false;
    fan = 0;
    fanRpmSensorPin = this->systemService->registerSysPin("heater", "fanRpmSensorPin");
    fanRpmSensorPin.setPin(21);
    fanRpmCounterUnit = PCNT_UNIT_3;
    fanRpm = 0;

    ServiceCommands *cmd = commandMgr->getServiceCommands("heater");
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

    // init hardware
    adc1_config_width(ADC_WIDTH_BIT_12);
    if (heaterTempAdcPin != -1) {
        heaterAdcChannel = gpioToChannel(heaterTempAdcPin);
        adc1_config_channel_atten(heaterAdcChannel, heaterAdcAtten);
        esp_adc_cal_characterize(ADC_UNIT_1, heaterAdcAtten, ADC_WIDTH_BIT_12, 1100, &heaterAdcCharacteristics);
    }
    if (airTempAdcPin != -1) {
        airAdcChannel = gpioToChannel(airTempAdcPin);
        adc1_config_channel_atten(airAdcChannel, airAdcAtten);
        esp_adc_cal_characterize(ADC_UNIT_1, airAdcAtten, ADC_WIDTH_BIT_12, 1100, &airAdcCharacteristics);
    }

    ledc_timer_config_t ledcTimer;
    ledcTimer.duty_resolution = LEDC_TIMER_11_BIT;
    ledcTimer.freq_hz = 25000;
    ledcTimer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledcTimer.timer_num = pwmTimer;
    rc = ledc_timer_config(&ledcTimer);

    if (heaterPwmPin != -1) {
        digitalWrite(heaterPwmPin, (heaterPwmIsInverted ? HIGH : LOW));
        pinMode(heaterPwmPin, OUTPUT);
        
        heaterPwmChannelConfig.channel = heaterPwmChannel;
        heaterPwmChannelConfig.gpio_num = heaterPwmPin.getPin();
        heaterPwmChannelConfig.speed_mode = LEDC_HIGH_SPEED_MODE;
        heaterPwmChannelConfig.timer_sel = pwmTimer;
        heaterPwmChannelConfig.intr_type = LEDC_INTR_DISABLE;
        heaterPwmChannelConfig.duty = (heaterPwmIsInverted ? 2048 : 0);
        heater = 0;

        rc = ledc_channel_config(&heaterPwmChannelConfig);
    }
    if (fanPwmPin != -1) {
        digitalWrite(fanPwmPin, (fanPwmIsInverted ? HIGH : LOW));
        pinMode(fanPwmPin, OUTPUT);

        fanPwmChannelConfig.channel = fanPwmChannel;
        fanPwmChannelConfig.gpio_num = fanPwmPin.getPin();
        fanPwmChannelConfig.speed_mode = LEDC_HIGH_SPEED_MODE;
        fanPwmChannelConfig.timer_sel = pwmTimer;
        fanPwmChannelConfig.intr_type = LEDC_INTR_DISABLE;
        fanPwmChannelConfig.duty = (fanPwmIsInverted ? 2048 : 0);
        fan = 0;

        rc = ledc_channel_config(&fanPwmChannelConfig);
Serial.printf("ledc_channel_config fan: %d\n", rc);
    }

    if (fanRpmSensorPin != -1) {
            // set up counter
        pcnt_config_t cfg;
        cfg.pulse_gpio_num = fanRpmSensorPin;
        cfg.ctrl_gpio_num = PCNT_PIN_NOT_USED;
        cfg.lctrl_mode = PCNT_MODE_KEEP;
        cfg.hctrl_mode = PCNT_MODE_KEEP;
        cfg.pos_mode = PCNT_COUNT_INC;  // count only rising edges
        cfg.neg_mode = PCNT_COUNT_DIS;
        cfg.counter_h_lim = 0;
        cfg.counter_l_lim = 0;
        cfg.unit = fanRpmCounterUnit;
        cfg.channel = PCNT_CHANNEL_0;
        esp_err_t rc = pcnt_unit_config(&cfg);

        // set the GPIO back to high-impedance / pulldown, as pcnt_unit_config sets it as pull-up
        // rc = gpio_set_pull_mode((gpio_num_t)fanRpmSensorPin, /* GPIO_PULLDOWN_ONLY */ GPIO_FLOATING);

        // enable counter filter - at 80MHz APB CLK, 1000 pulses is max 80,000 Hz, so ignore pulses less than 12.5 us.
        int filterLength = 65535;
        rc = pcnt_set_filter_value(fanRpmCounterUnit, filterLength);
        if (filterLength > 0) {
            rc = pcnt_filter_enable(fanRpmCounterUnit);
        } else {
            rc = pcnt_filter_disable(fanRpmCounterUnit);
        }

        fanRpmCounterTimer.init(eventLoop, [this](UEventLoopTimer *timer) {
            uint16_t counterCount = 0;
            int64_t tm = esp_timer_get_time();
            esp_err_t rc = pcnt_get_counter_value(fanRpmCounterUnit, (int16_t*)&counterCount);
            pcnt_counter_clear(fanRpmCounterUnit);
            logger->debug("Counter value: {}", counterCount);
            if (rc != ESP_OK) {
                fanRpm = -1;
            } else {
                fanRpm = (int)((int32_t)counterCount * 60 * 1000 / ((tm - fanRpmLastTime) / 1000));
            }
            fanRpmLastTime = tm;
        });
        rc = pcnt_counter_clear(fanRpmCounterUnit);
        fanRpm = 0;
        fanRpmLastTime = esp_timer_get_time();
        fanRpmCounterTimer.setInterval(1000); // at 1500 rpm, we'll get 25 counts per second

    }

}

void HeaterService::setHeaterDuty(int duty)
{
    int dutyVal = heaterPwmIsInverted ? ((1000 - duty) * 2048 + 500) / 1000 : (duty * 2048 + 500) / 1000;
    esp_err_t rc = ledc_set_duty(heaterPwmChannelConfig.speed_mode, heaterPwmChannelConfig.channel, dutyVal);
    rc = ledc_update_duty(heaterPwmChannelConfig.speed_mode, heaterPwmChannelConfig.channel);
}

void HeaterService::setFanDuty(int duty)
{
    int dutyVal = fanPwmIsInverted ? ((1000 - duty) * 2048 + 500) / 1000 : (duty * 2048 + 500) / 1000;
    esp_err_t rc = ledc_set_duty(fanPwmChannelConfig.speed_mode, fanPwmChannelConfig.channel, dutyVal);
    rc = ledc_update_duty(fanPwmChannelConfig.speed_mode, fanPwmChannelConfig.channel);
}

int HeaterService::readAdc(adc1_channel_t channel, esp_adc_cal_characteristics_t *characteristics, int samples)
{
    int16_t raw[samples];
    for (int i = 0; i < samples; i++) {
        raw[i] = adc1_get_raw(channel);
    }
    int64_t sumX = 0;
    int64_t sumX2 = 0;
    for (int i = 0; i < samples; i++) {
        sumX += raw[i];
        sumX2 += raw[i] * raw[i];
    }
    sumX *= 256;
    sumX2 *= 256 * 256;
    int v = (int)(sumX / samples);
    int variance = (sumX2 / samples - sumX * sumX / samples / samples);
    int64_t sum2 = 0;
    int n = 0;
    for (int i = 0; i < samples; i++) { // keep within 1 stddev
        if ((raw[i] * 256 - v) * (raw[i] * 256 - v) <= variance) { // we want to avoid sqrt() in stddev = sqrt(variance)
            sum2 += raw[i];
            ++n;
        }
    }
    logger->debug("ADC on channel {}, avg {}, stddev {}, n = {}\n", channel, v / 256, sqrt(variance) / 256, n);
    int v2 = (n == 0 ? 0 : sum2 * 256 / n);

    uint64_t voltageTemp = ((((uint64_t)characteristics->coeff_a * v2 + 65536/2 * 256) + (uint64_t)characteristics->coeff_b * 65536 * 256) / 65536 + 256/2);
    int voltage = (int)(voltageTemp / 256);
    return voltage;
}

void HeaterService::initCommands(ServiceCommands *cmd)
{
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&heaterPwmPin)
        .help("--> Pin for controlling the heater via PWM (requires reboot after change)")
    );
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&fanPwmPin)
        .help("--> Pin for controlling the fan via PWM (requires reboot after change)")
    );
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&fanRpmSensorPin)
        .help("--> Pin for fan RPM sensor (requires reboot after change)")
    );
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&heaterTempAdcPin)
        .help("--> ADC pin for heater temperature reading (requires reboot after change)")
    );
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&airTempAdcPin)
        .help("--> ADC pin for air temperature reading (requires reboot after change)")
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("heaterAdcOversampling", true)
        .cmd("heaterAdcOversampling")
        .help("--> Heater ADC number of samples for averaging")
        .vMin(1)
        .vMax(32768)
        .ptr(&heaterAdcOversampling)
    );
    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("heaterAdcAttenuation", true)
        .cmd("heaterAdcAttenuation")
        .help("--> heater ADC attenuation in dB (0, 2.5, 6, 11)")
        .getFn([this](String *val) {
            *val = (heaterAdcAtten == ADC_ATTEN_0db ? "0"
                : heaterAdcAtten == ADC_ATTEN_2_5db ? "2.5"
                : heaterAdcAtten == ADC_ATTEN_6db ? "6"
                : heaterAdcAtten == ADC_ATTEN_11db ? "11"
                : "?");
        })
        .setFn([this](const String &val, bool isLoading, String *msg) {
            if (val == "0") {
                heaterAdcAtten = ADC_ATTEN_0db;
            } else if (val == "2.5") {
                heaterAdcAtten = ADC_ATTEN_2_5db;
            } else if (val == "6") {
                heaterAdcAtten = ADC_ATTEN_6db;
            } else if (val == "11") {
                heaterAdcAtten = ADC_ATTEN_11db;
            } else {
                *msg = "Unrecognized attenuation, expected 0, 2.5, 6 or 11";
                return true;
            }
            if (!isLoading) {
                adc1_config_channel_atten(heaterAdcChannel, heaterAdcAtten);
                esp_adc_cal_characterize(ADC_UNIT_1, heaterAdcAtten, ADC_WIDTH_BIT_12, 1100, &heaterAdcCharacteristics);
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("heaterAdc", true)
        .cmd("heaterAdc")
        .help("--> Read heater ADC")
        .isPersistent(false)
        .getFn([this] {
            int v = readAdc(heaterAdcChannel, &heaterAdcCharacteristics, heaterAdcOversampling);
            return v;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("heater", true)
        .cmd("heater")
        .help("--> Set or get heater, scale 0..1000")
        .isPersistent(false)
        .vMin(0)
        .vMax(1000)
        .getFn([this] { return heater; })
        .setFn([this](int val, bool isLoading, String *msg) {
            setHeaterDuty(val);
            heater = val;
            *msg = String("Heater set to ") + heater;
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("fan", true)
        .cmd("fan")
        .help("--> Set or get fan, scale 0..1000")
        .isPersistent(false)
        .vMin(0)
        .vMax(1000)
        .getFn([this] { return fan; })
        .setFn([this](int val, bool isLoading, String *msg) {
            setFanDuty(val);
            fan = val;
            *msg = String("Fan set to ") + fan;
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("fanRpm", true)
        .cmd("fanRpm")
        .help("--> Current fan RPM")
        .isPersistent(false)
        .getFn([this] { return fanRpm; })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("airAdcOversampling", true)
        .cmd("airAdcOversampling")
        .help("--> Air ADC number of samples for averaging")
        .vMin(1)
        .vMax(32768)
        .ptr(&airAdcOversampling)
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("airAdcAttenuation", true)
        .cmd("airAdcAttenuation")
        .help("--> Air ADC attenuation in dB (0, 2.5, 6, 11)")
        .getFn([this](String *val) {
            *val = (airAdcAtten == ADC_ATTEN_0db ? "0"
                : airAdcAtten == ADC_ATTEN_2_5db ? "2.5"
                : airAdcAtten == ADC_ATTEN_6db ? "6"
                : airAdcAtten == ADC_ATTEN_11db ? "11"
                : "?");
        })
        .setFn([this](const String &val, bool isLoading, String *msg) {
            if (val == "0") {
                airAdcAtten = ADC_ATTEN_0db;
            } else if (val == "2.5") {
                airAdcAtten = ADC_ATTEN_2_5db;
            } else if (val == "6") {
                airAdcAtten = ADC_ATTEN_6db;
            } else if (val == "11") {
                airAdcAtten = ADC_ATTEN_11db;
            } else {
                *msg = "Unrecognized attenuation, expected 0, 2.5, 6 or 11";
                return true;
            }
            if (!isLoading) {
                adc1_config_channel_atten(airAdcChannel, airAdcAtten);
                esp_adc_cal_characterize(ADC_UNIT_1, airAdcAtten, ADC_WIDTH_BIT_12, 1100, &airAdcCharacteristics);
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("airAdc", true)
        .cmd("airAdc")
        .help("--> Read air ADC")
        .isPersistent(false)
        .getFn([this] {
            int v = readAdc(airAdcChannel, &airAdcCharacteristics, airAdcOversampling);
            return v;
        })
    );

}

#endif
