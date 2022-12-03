#include <CompilationOpts.h>

#if defined(USE_GPSDO) && defined(GPSDO_GEN)

#include <HardwareSerial.h>
#include <driver/ledc.h>
#include <soc/ledc_reg.h>
#include <soc/ledc_struct.h>
#include <driver/rmt.h>
#include <freertos/FreeRTOS.h>

#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "SystemService.h"
#include "MqttService.h"
#include "GpsdoGen.h"

void GpsdoGen::init(const char *serviceName, UEventLoop *eventLoop, SystemService *systemService,
    MqttService *mqtt, ServiceCommands *cmd, Logger *logger)
{
    this->eventLoop = eventLoop;
    this->logger = logger;
    this->mqtt = mqtt;

    isGenEnabled = false;
    genPpsPin = systemService->registerSysPin(serviceName, "gen.ppsPin");
    genPulsePin = systemService->registerSysPin(serviceName, "gen.pulsePin");
    genPulseFreq = 100000;

    initCommands(cmd);
}

void GpsdoGen::postInit()
{
    if (genPulsePin != -1) {
        pinMode(genPulsePin.getPin(), OUTPUT);
        digitalWrite(genPulsePin, LOW);
    }
    if (isGenEnabled) {
        isGenEnabled = false;
        enableGen();
    }
}

void GpsdoGen::enableGen()
{
    if (isGenEnabled) {
        return;
    }
    if (genPpsPin.getPin() > 0) {
        // to create a precise 1pps signal we use the RMT peripheral (as with PWM we can go no lower than 125 Hz with ABP clock)
        rmt_config_t rmtCfg {
            .rmt_mode = RMT_MODE_TX,
            .channel = RMT_CHANNEL_2,
            .clk_div = 80,
            .gpio_num = (gpio_num_t)genPpsPin.getPin(),
            .mem_block_num = 1,
            {
                .tx_config = {
                    .loop_en = true,
                    .carrier_freq_hz = 0,
                    .carrier_duty_percent = 0,
                    .carrier_level = RMT_CARRIER_LEVEL_LOW,
                    .carrier_en = false,
                    .idle_level = RMT_IDLE_LEVEL_LOW,
                    .idle_output_en = true
                }
            }
        };
        esp_err_t rc = rmt_config(&rmtCfg);
        if (rc != ESP_OK) {
            logger->error("Error in rmt_config(): {}", rc);
        }
        rc = rmt_driver_install(RMT_CHANNEL_2, 0, 0);
        if (rc != ESP_OK) {
            logger->error("Error in rmt_driver_install(): {}\n", rc);
        }

        rmt_item32_t items[64];
        for (int i = 0; i < 64; i++) {
            items[i].duration0 = 15625 / 10;
            items[i].level0 = ((i / 16) % 2);
            items[i].duration1 = 15625 / 10;
            items[i].level1 = ((i / 16) % 2);
        }
        rc = rmt_fill_tx_items(RMT_CHANNEL_2, items, 64, 0);
        if (rc != ESP_OK) {
            logger->error("Error in rmt_fill_tx_items(): {}\n", rc);
        }
        rc = rmt_tx_start(RMT_CHANNEL_2, true);
        if (rc != ESP_OK) {
            logger->error("Error in rmt_tx_start(): {}\n", rc);
        }
        logger->info("rmt config is OK");

    }

    if (genPulsePin.getPin() > 0) {
        ledcTimerPulse = {
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            { .duty_resolution = (ledc_timer_bit_t)LEDC_TIMER_1_BIT },
            .timer_num = LEDC_TIMER_1,
            .freq_hz = (unsigned)genPulseFreq
        };
        esp_err_t rc = ledc_timer_config(&ledcTimerPulse);
        if (rc != ESP_OK) {
            logger->error("Error configuring pwm timer for pulse");
        }

        ledcChannelPulse = {
            .gpio_num = genPulsePin,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .channel = LEDC_CHANNEL_1,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_1,
            .duty = 1,
            .hpoint = 0
        };
        rc = ledc_channel_config(&ledcChannelPulse);
        if (rc != ESP_OK) {
            logger->error("Error configuring pwm channel for pulse");
        }
    }
    isGenEnabled = true;

#ifdef USE_MQTT
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"enabled\":\"true\",\"pps\":\"true\",\"pulseFreq\":%d}", genPulseFreq);
    buf[sizeof(buf) - 1] = '\0';
    mqtt->publishTele("gpsdo/genStatus", 0, true, buf);
#endif
}

void GpsdoGen::disableGen()
{
    if (!isGenEnabled) {
        return;
    }
    rmt_tx_stop(RMT_CHANNEL_2);
    ledc_stop(ledcChannelPulse.speed_mode, ledcChannelPulse.channel, 0);
#ifdef USE_MQTT
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"enabled\":\"false\"}");
    buf[sizeof(buf) - 1] = '\0';
    mqtt->publishTele("gpsdo/genStatus", 0, true, buf);
#endif
    isGenEnabled = false;
}

bool GpsdoGen::setGenPulseFreq(int freq)
{
    uint32_t div = 10240000000ull / (uint32_t)freq;
    if (isGenEnabled) {
        esp_err_t rc = ledc_timer_set(ledcChannelPulse.speed_mode, ledcChannelPulse.timer_sel, div, 1, LEDC_APB_CLK);
Serial.printf("setGenPulseFreq(%d), div = %d+%d*256, rc = %d\n", freq, div / 256, div & 0xFF, rc);
        if (rc != ESP_OK) {
            return false; // and don't do mqtt
        }
    }
    genPulseFreq = freq;
    genPulseDiv = div;

#ifdef USE_MQTT
    char buf[128];
    snprintf(buf, sizeof(buf), "{\"enabled\":\"%s\",\"pps\":\"true\",\"pulseFreq\":%d}", isGenEnabled ? "true" : "false", genPulseFreq);
    buf[sizeof(buf) - 1] = '\0';
    mqtt->publishTele("gpsdo/genStatus", 0, true, buf);
#endif
    return true;
}

void GpsdoGen::initCommands(ServiceCommands *cmd)
{
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&genPpsPin)
        .help("--> Set output pin for generating 1 pps signal - requires reboot")
    );
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&genPulsePin)
        .help("--> Set output pin for generating the configurable frequency signal - requires reboot")
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("gen.pulseFreq", true)
        .cmd("gen.pulseFreq")
        .help("--> Frequency of the generated signal")
        .vMin(1)
        .vMax(40000000)
        .getFn([this]() {
            return genPulseFreq;
        })
        .setFn([this](int val, bool isLoading, String *msg) {
            if (isLoading) {
                genPulseFreq = val;
                *msg = "LOADING";
                return true;
            }
            bool rc = setGenPulseFreq(val);
            if (rc) {
                *msg = "Set genPulse frequency to "; msg->concat(genPulseFreq);
            } else {
                *msg = "Error trying to set genPulse frequency";
            }
            return true;
        })
    );


    cmd->registerIntData(ServiceCommands::IntDataBuilder("gen.pulseFreq_readHardware", true)
        .cmd("gen.pulseFreq_readHardware")
        .help("--> Frequency of the generated signal, reading hardware")
        .isPersistent(false)
        .getFn([this]() {
            if (isGenEnabled) {
                return (int) ledc_get_freq(ledcChannelPulse.speed_mode, ledcChannelPulse.timer_sel);
            } else {
                return 0;
            }
        })
    );

    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("gen.enabled", true)
        .cmdOn("gen.enable")
        .cmdOff("gen.disable")
        .helpOn("--> Enable the test signal generation")
        .helpOff("--> Disable the test signal generation")
        .ptr(&isGenEnabled)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (isLoading) {
                isGenEnabled = val;
                return true;
            }
            if (val == isGenEnabled) {
                *msg = String("Signal generation already ") + (isGenEnabled ? "enabled" : "disabled");
            } else {
                if (val) {
                    enableGen();
                } else {
                    disableGen();
                }
                *msg = (isGenEnabled ? "Signal generation enabled" : "Signal generation disabled");
            }
            return true;
        })
    );

}

#endif