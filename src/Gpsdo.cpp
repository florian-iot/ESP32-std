#include <CompilationOpts.h>

#ifdef USE_GPSDO

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "driver/periph_ctrl.h"
#include <driver/pcnt.h>
#include "driver/gpio.h"
#include <driver/ledc.h>
#include <esp_intr_alloc.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Gpsdo.h"


static DRAM_ATTR UEventLoop::IsrData isrData;
static DRAM_ATTR volatile uint32_t ppsEventType;
static uint32_t volatile DRAM_ATTR overflowCount[3];


void GpsdoFreqCounterService::init(UEventLoop *eventLoop, SystemService *systemService,
#ifdef USE_MQTT
    MqttService *mqtt,
#endif
    CommandMgr *commandMgr, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
#ifdef USE_MQTT
    this->mqtt = mqtt;
#endif
    const char *serviceName = "gpsdo";
    this->logger = logMgr->newLogger(serviceName);

    // defaults
    isEnabled = false;
    ppsPin = systemService->registerSysPin(serviceName, "ppsPin");
    pulsePin = systemService->registerSysPin(serviceName, "pulsePin");
    flipFlopDataPin = systemService->registerSysPin(serviceName, "flipFlopDataPin");
    ctrl1Pin = systemService->registerSysPin(serviceName, "ctrl1Pin");
    ctrl2Pin = systemService->registerSysPin(serviceName, "ctrl2Pin");
    filterLength = 0;
    gateSeconds = 1;

    gateSecondsCount = 0;
    counter = 0;
    ppsCounter = 0;
    ppsCounterCumulative = 0;
    errorDuration = 0;
    error = 0;

    ServiceCommands *cmd = commandMgr->getServiceCommands(serviceName);
    initPpsDfa();
//    initMainDfa();
    initCommands(cmd);

    dac.init(serviceName, eventLoop, systemService, cmd, logger);
    gen.init(serviceName, eventLoop, systemService, mqtt, cmd, logger);

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
    isPpsPresent = false;
    ppsTimestamp = 0;
    firstPpsCount = 0;

    eventLoop->initIsrData(&isrData);
    ppsEventType = eventLoop->getEventType("gpsdo", "pps");

    ppsTimeoutTimer.init(eventLoop, [this](UEventLoopTimer *timer) {
Serial.printf("Gpsdo timed out waiting for pps signal (no pps for %lld micros)\n", esp_timer_get_time() - ppsTimestamp);
        firstPpsCount = 0;
        if (isPpsPresent) {
            isPpsPresent = false; // so that ppsDfa finds isPpsPresent false
            ppsDfa.handleInput(FC_PPS_SYNC_LOST);
        }
    });

    eventLoop->onEvent(ppsEventType, [this](UEvent *event) {
        // happens at every pps
        // we want to tell this to DFA
        int64_t ts = event->dataInt;

        int64_t timeDiff = ts - ppsTimestamp;
        ppsTimestamp = ts;
Serial.printf("Gpsdo pps at %lld, diff with previous %lld\n", ts, timeDiff);

        if (firstPpsCount == 0 || (timeDiff > 800000 && timeDiff < 1200000)) {
if (firstPpsCount == 0) {
    Serial.printf("Gpsdo got pps signal, expecting some more before considering pps as locked\n");
}
            if (firstPpsCount < 8) {
                ++firstPpsCount; // don't feed to ppsDfa until we're sure we've locked pps timing
            } else {
if (!isPpsPresent) { // print out only the first time
    Serial.printf("Gpsdo locked pps signal\n");
}
                isPpsPresent = true;
                ppsDfa.handleInput(FC_PPS);
            }
        } else { // not looking a good pps
            firstPpsCount = 0;
            if (isPpsPresent) {
Serial.printf("Gpsdo lost lock of pps signal (pps with time from previous pps: %lld micros)\n", timeDiff);
                isPpsPresent = false; // so that ppsDfa finds isPpsPresent false
                ppsDfa.handleInput(FC_PPS_SYNC_LOST);
            } else {
Serial.printf("Gpsdo still not locking pps signal (pps with time from previous pps: %lld micros)\n", timeDiff);
            }
        }
        ppsTimeoutTimer.setTimeout(1300); // if it times out, we've lost a pps
        return true;
    });

    dac.postInit();
    gen.postInit();

    if (isEnabled) {
        isEnabled = false;
        enable(); // will start the pps DFA
    }
}

static IRAM_ATTR void pps_intr_handler(void *arg)
{
    UEvent evt(ppsEventType, esp_timer_get_time());
    isrData.queueEventFromIsr(evt);
}

static IRAM_ATTR void pcnt_intr_handler(void *arg)
{
    uint32_t intr_status = PCNT.int_st.val;
    overflowCount[0] += (intr_status & (1 << PCNT_UNIT_LONG_GATE));
    overflowCount[1] += ((intr_status & (1 << PCNT_UNIT_PPS_RAISING)) >> 1);
    overflowCount[2] += ((intr_status & (1 << PCNT_UNIT_PPS_FALLING)) >> 2);
    PCNT.int_clr.val |= intr_status & ((1 << PCNT_UNIT_LONG_GATE) | (1 << PCNT_UNIT_PPS_RAISING) | (1 << PCNT_UNIT_PPS_FALLING)) ;
}

void GpsdoFreqCounterService::enable()
{
    if (isEnabled) {
        return;
    }
    if (ppsPin == -1 || flipFlopDataPin == -1 || pulsePin == -1 || ctrl1Pin == -1 || ctrl2Pin == -1) {
        return;
    }
    overflowCount[0] = 0;
    overflowCount[1] = 0;
    overflowCount[2] = 0;
    pinMode(ppsPin, INPUT_PULLDOWN);
    digitalWrite(flipFlopDataPin, LOW);
    pinMode(flipFlopDataPin, OUTPUT);
    digitalWrite(flipFlopDataPin, LOW);
    initCounter(PCNT_UNIT_LONG_GATE, PCNT_CHANNEL_0, &pcntLongGateConfig, pulsePin, ctrl1Pin, false);
    initCounter(PCNT_UNIT_PPS_RAISING, PCNT_CHANNEL_0, &pcntPpsRaisingConfig, pulsePin, ctrl2Pin, false);
    initCounter(PCNT_UNIT_PPS_FALLING, PCNT_CHANNEL_0, &pcntPpsFallingConfig, pulsePin, ctrl2Pin, true);
    pcnt_counter_clear(PCNT_UNIT_LONG_GATE);
    pcnt_counter_clear(PCNT_UNIT_PPS_RAISING);
    pcnt_counter_clear(PCNT_UNIT_PPS_FALLING);

    ESP_ERROR_CHECK(pcnt_isr_register(pcnt_intr_handler, NULL, ESP_INTR_FLAG_IRAM, &isr_handle));
    ESP_ERROR_CHECK(pcnt_event_enable(PCNT_UNIT_LONG_GATE, PCNT_EVT_THRES_0));
    ESP_ERROR_CHECK(pcnt_event_enable(PCNT_UNIT_PPS_RAISING, PCNT_EVT_THRES_0));
    ESP_ERROR_CHECK(pcnt_event_enable(PCNT_UNIT_PPS_FALLING, PCNT_EVT_THRES_0));
    ESP_ERROR_CHECK(pcnt_intr_enable(PCNT_UNIT_LONG_GATE));
    ESP_ERROR_CHECK(pcnt_intr_enable(PCNT_UNIT_PPS_RAISING));
    ESP_ERROR_CHECK(pcnt_intr_enable(PCNT_UNIT_PPS_FALLING));

    esp_err_t rc = gpio_install_isr_service(0/*ESP_INTR_FLAG_IRAM*/);
    if (rc != 0 && rc != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(rc);
    }
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)ppsPin.getPin(), pps_intr_handler, nullptr));
    ESP_ERROR_CHECK(gpio_set_intr_type((gpio_num_t)ppsPin.getPin(), GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_intr_enable((gpio_num_t)ppsPin.getPin()));

    isEnabled = true;

    // start the pps DFA (noop if already started)
    ppsDfa.handleInput(FC_START);
}

void GpsdoFreqCounterService::disable()
{
    if (!isEnabled) {
        return;
    }
    gpio_isr_handler_remove((gpio_num_t)ppsPin.getPin());
    esp_intr_free(isr_handle);
    pinMode(ppsPin, INPUT);
    pinMode(flipFlopDataPin, INPUT);
    pinMode(pulsePin, INPUT);
    pinMode(ctrl1Pin, INPUT);
    pinMode(ctrl2Pin, INPUT);

    isEnabled = false;
}

void GpsdoFreqCounterService::initCounter(pcnt_unit_t unit, pcnt_channel_t channel, pcnt_config_t *cfg,
    int pulsePin, int ctrlPin, bool isCtrlPinInverted)
{
    if (unit == -1 || pulsePin == -1 || ctrlPin == -1) {
        return;
    }
    // set up counter
    *cfg = {
        .pulse_gpio_num = pulsePin,
        .ctrl_gpio_num = ctrlPin,
        .lctrl_mode = isCtrlPinInverted ? PCNT_MODE_KEEP : PCNT_MODE_DISABLE,
        .hctrl_mode = isCtrlPinInverted ? PCNT_MODE_DISABLE : PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC,  // count only rising edges
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = 0,
        .counter_l_lim = 0,
        .unit = unit,
        .channel = channel,
    };
    ESP_ERROR_CHECK(pcnt_unit_config(cfg));

    // set the GPIO back to high-impedance / pulldown, as pcnt_unit_config sets it as pull-up
    ESP_ERROR_CHECK(gpio_set_pull_mode((gpio_num_t)pulsePin, /* GPIO_PULLDOWN_ONLY */ GPIO_FLOATING));

    // enable counter filter - at 80MHz APB CLK, 1000 pulses is max 80,000 Hz, so ignore pulses less than 12.5 us.
    ESP_ERROR_CHECK(pcnt_set_filter_value(unit, filterLength));
    if (filterLength > 0) {
        ESP_ERROR_CHECK(pcnt_filter_enable(unit));
    } else {
        ESP_ERROR_CHECK(pcnt_filter_disable(unit));
    }
}

int64_t GpsdoFreqCounterService::retrievePpsCounter()
{
    int ppsPinStatus = digitalRead(ppsPin);
    pcnt_unit_t cUnit = (ppsPinStatus == HIGH ? PCNT_UNIT_PPS_RAISING : PCNT_UNIT_PPS_FALLING);
    int c = (ppsPinStatus == HIGH ? 1 : 2);

    // We're sure that overflowCount[c] is not being modified, because
    // this counter is not counting now. It was counting last period, now it's the other counter that's counting.

    uint32_t overflowCountRetrieved = overflowCount[c];
    uint16_t counterCount = 0;
    ESP_ERROR_CHECK(pcnt_get_counter_value(cUnit, (int16_t*)&counterCount));
    int64_t counter = (int64_t)overflowCountRetrieved * 0x10000 + (int64_t)counterCount;

    overflowCount[c] = 0;
    ESP_ERROR_CHECK(pcnt_counter_clear(cUnit));

    return counter;
}

int64_t GpsdoFreqCounterService::retrieveLongGateCounter()
{
    // counter 0 is not counting right now, we've set the gate to low
    uint32_t overflowCountRetrieved = overflowCount[0];
    uint16_t counterCount = 0;
    ESP_ERROR_CHECK(pcnt_get_counter_value(PCNT_UNIT_LONG_GATE, (int16_t*)&counterCount));
    int64_t counter = (int64_t)overflowCountRetrieved * 0x10000 + (int64_t)counterCount;

    overflowCount[0] = 0;
    ESP_ERROR_CHECK(pcnt_counter_clear(PCNT_UNIT_LONG_GATE));

    return counter;
}

#ifdef USE_MQTT
void GpsdoFreqCounterService::publishStatus(const char *reason)
{
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "{\"enabled\":\"%s\",\"pps\":%s,\"ppsReason\":\"%s\",\"dac\":%d,\"dacMax\":%d",
        isEnabled ? "true" : "false", isPpsPresent ? "true" : "false", reason, dac.getValue(), dac.getMaxValue());

    strncat(buf + n, "}", sizeof(buf) - n);
    buf[sizeof(buf) - 1] = '\0';
    mqtt->publishTele("gpsdo/status", 0, true, buf);
}
#else
void GpsdoFreqCounterService::publishStatus(const char *reason)
{
}
#endif

void GpsdoFreqCounterService::initPpsDfa()
{   
    ppsDfa.init(eventLoop, nullptr /* or logger, to log */, FC_STARTUP);

    ppsDfa.onInput([this](Dfa *dfa, Dfa::State state, Dfa::Input input) {

Serial.printf("Gpsdo at %lu, DFA State %s, input %s\n", millis(), dfa->stateName(state), dfa->inputName(input));

        if (state.is(FC_STARTUP)) {

            if (input.is(FC_START)) {
                publishStatus("Startup");
                return dfa->transitionTo(FC_WAIT_FIRST_PPS);
            }

        } else if (state.is(FC_WAIT_BEFORE_FIRST_PPS)) {

            if (input.is(FC_PPS)) {
                return dfa->transitionTo(FC_WAIT_BEFORE_FIRST_PPS_PAST, PAST_MILLIS);
            } else if (input.is(FC_PPS_SYNC_LOST)) {
                return dfa->transitionTo(FC_PPS_LOST);
            }

        } else if (state.is(FC_WAIT_BEFORE_FIRST_PPS_PAST)) {

            if (input.is(Dfa::Input::TIMEOUT)) { // we're at some ms after the first PPS after start command
                // ensure we're less than 500 ms after last pps - else we're too late, we need to skip this pps
                if (esp_timer_get_time() - ppsTimestamp > 500000) {
                    return dfa->transitionTo(FC_PPS_LOST);
                }
                // clear counters
                (void) retrieveLongGateCounter();
                (void) retrievePpsCounter();
                return dfa->transitionTo(FC_WAIT_FIRST_PPS);
            } else if (input.is(FC_PPS, FC_PPS_SYNC_LOST)) {
                return dfa->transitionTo(FC_PPS_LOST);
            }

        } else if (state.is(FC_WAIT_FIRST_PPS)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                digitalWrite(flipFlopDataPin, LOW);
                return dfa->noTransition();
            } else if (input.is(FC_PPS)) {
                return dfa->transitionTo(FC_WAIT_FIRST_PPS_PAST, PAST_MILLIS);
            } else if (input.is(FC_PPS_SYNC_LOST)) {
                return dfa->transitionTo(FC_PPS_LOST);
            }

        } else if (state.is(FC_WAIT_FIRST_PPS_PAST)) {

            if (input.is(Dfa::Input::TIMEOUT)) { // we're at some ms after the first PPS after start command
                // ensure we're less than 500 ms after last pps - else we're too late, we need to skip this pps
                if (esp_timer_get_time() - ppsTimestamp > 500000) {
                    return dfa->transitionTo(FC_PPS_LOST);
                }
                // clear counters
                (void) retrieveLongGateCounter();
                (void) retrievePpsCounter();
                counter = 0;
                ppsCounter = 0;
                ppsCounterCumulative = 0;
                errorDuration = 0;
                error = 0;
                publishStatus("Got pps");
                // set flip flop data, so that next pps sets counter gate to high
                digitalWrite(flipFlopDataPin, HIGH);
                // how many seconds will be the gate time
                gateSecondsCount = gateSeconds;
                return dfa->transitionTo(FC_WAIT_NEXT_PPS);
            } else if (input.is(FC_PPS, FC_PPS_SYNC_LOST)) {
                return dfa->transitionTo(FC_PPS_LOST);
            }

        } else if (state.is(FC_WAIT_NEXT_PPS)) {

            if (input.is(FC_PPS)) {
                return dfa->transitionTo(FC_WAIT_NEXT_PPS_PAST, PAST_MILLIS);
            } else if (input.is(FC_PPS_SYNC_LOST)) {
                return dfa->transitionTo(FC_PPS_LOST);
            }

        } else if (state.is(FC_WAIT_NEXT_PPS_PAST)) {

            if (input.is(Dfa::Input::TIMEOUT)) {
                // ensure we're less than 500 ms after last pps - else we're too late, we need to skip this pps
                if (esp_timer_get_time() - ppsTimestamp > 500000) {
                    return dfa->transitionTo(FC_PPS_LOST);
                }
                ppsCounter = retrievePpsCounter();

                ++errorDuration;
                error += (int32_t)(ppsCounter - 10000000);

                // count in cumulative pps counter only after the first period, because we activated the gate only for the first period
                if (gateSecondsCount < gateSeconds) {
                    ppsCounterCumulative += ppsCounter;
                }

                Serial.printf("ppsCounter: %lld\n", ppsCounter);
#ifdef USE_MQTT
                char buf[128];
                snprintf(buf, sizeof(buf), "{\"ppsCounter\":%lld,\"error\":%d,\"errorDuration\":%d,\"dac\":%d}",
                    ppsCounter, error, errorDuration, dac.getValue());
                buf[sizeof(buf) - 1] = '\0';
                mqtt->publishTele("gpsdo/ppsCounter", 0, false, buf);
#endif
                --gateSecondsCount;
                if (gateSecondsCount == 0) {
                    // set flip flop data, so that next pps sets counter gate to low and we stop counting
                    digitalWrite(flipFlopDataPin, LOW);
                    return dfa->transitionTo(FC_WAIT_LAST_PPS);
                } else {
                    return dfa->transitionTo(FC_WAIT_NEXT_PPS);
                }
            } else if (input.is(FC_PPS, FC_PPS_SYNC_LOST)) {
                return dfa->transitionTo(FC_PPS_LOST);
            }

        } else if (state.is(FC_WAIT_LAST_PPS)) {

            if (input.is(FC_PPS)) {
                return dfa->transitionTo(FC_WAIT_LAST_PPS_PAST, PAST_MILLIS);
            } else if (input.is(FC_PPS_SYNC_LOST)) {
                return dfa->transitionTo(FC_PPS_LOST);
            }

        } else if (state.is(FC_WAIT_LAST_PPS_PAST)) {
            
            if (input.is(Dfa::Input::TIMEOUT)) {
                // ensure we're less than 500 ms after last pps - else we're too late, we need to skip this pps
                if (esp_timer_get_time() - ppsTimestamp > 500000) {
                    return dfa->transitionTo(FC_PPS_LOST);
                }
                counter = retrieveLongGateCounter();
                ppsCounter = retrievePpsCounter();

                ppsCounterCumulative += ppsCounter;
                Serial.printf("ppsCounter: %lld\n", ppsCounter);

                // update error, in case we lost some pps
                error += (counter - ppsCounterCumulative);

                // the gate was set low, the counter is not counting anymore. Get its value now.
                Serial.printf("Counter gated at %d seconds: %lld\n", gateSeconds, counter);
#ifdef USE_MQTT
                char buf[128];
                snprintf(buf, sizeof(buf), "{\"counter\":%lld,\"gateSeconds\":%d,\"ppsCounterCumulative\":%lld}", counter, gateSeconds, ppsCounterCumulative);
                buf[sizeof(buf) - 1] = '\0';
                mqtt->publishTele("gpsdo/counter", 0, false, buf);
#endif

                // and we restart counting
                counter = 0;
                ppsCounter = 0;
                ppsCounterCumulative = 0;

                // set flip flop data, so that next pps sets counter gate to high
                digitalWrite(flipFlopDataPin, HIGH);
                // how many seconds will be the gate time
                gateSecondsCount = gateSeconds;
                return dfa->transitionTo(FC_WAIT_NEXT_PPS);
            } else if (input.is(FC_PPS, FC_PPS_SYNC_LOST)) {
                return dfa->transitionTo(FC_PPS_LOST);
            }

        } else if (state.is(FC_PPS_LOST)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                digitalWrite(flipFlopDataPin, LOW);
#ifdef USE_MQTT
                if (!isPpsPresent) {
                    publishStatus("Lost pps");
                } else {
                    isPpsPresent = false;
                    publishStatus("Processing delays caused lost pps lock");
                }
#endif
                return dfa->transitionTo(FC_WAIT_BEFORE_FIRST_PPS);
            }

        }

        // else
        return dfa->transitionError();
    });
}

// void GpsdoFreqCounterService::initMainDfa()
// {
//     mainDfa.init(eventLoop, nullptr /* or logger, to log */, FC_STARTUP);

    /*******************
     * 
     * Main logic
     * 
     * Upon startup
     * 
     * Set DAC value to the last know one, or to middle point. Wait DAC stabilization time.
     * 
     * Wait for pps signal to exist.
     * 
     * Check that the frequency is stable : verify that a linear regression of measures has
     * a slope close to 0 for a significant period, and there's no more than 1 count deviation
     * from average.
     * 
     * If no calibration found, calibrate.
     * 
     * Calibration:
     *     Calculate the frequency change for a given number of DAC value change, at 3 different
     *     DAC values.
     * 
     * Get current frequency during last N seconds. Calculate the error with 10MHz. Calculate expected
     * error (ex.: 2 * stddev), assuming normal distribution of each measure with known
     * stddev (50 ns). If error is outside of expected error, calculate the necessary DAC value
     * change to correct the error. Apply it, or 1/2 of it, or some other portion of it?
     * 
     */


//     mainDfa.onInput([this](Dfa *dfa, Dfa::State state, Dfa::Input input) {
//         return dfa->transitionError();
//     });
// }

void GpsdoFreqCounterService::initCommands(ServiceCommands *cmd)
{
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&ppsPin)
        .help("--> Set input pin for the pps signal")
    );
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&pulsePin)
        .help("--> Set input pin for the 10 MHz pulse from OCXO - requires reboot")
    );
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&flipFlopDataPin)
        .help("--> Set output pin for the flip-flop data pin - requires reboot")
    );
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&ctrl1Pin)
        .help("--> Set input pin for counter 1 gate - requires reboot")
    );
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&ctrl2Pin)
        .help("--> Set input pin for counter 2a and counter2b gate - requires reboot")
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("filterLength", true)
        .cmd("filterLength")
        .help("--> Length of filter for removing glitches in the counter - requires reboot")
        .vMin(0)
        .vMax(0xFFFF)
        .ptr(&filterLength)
    );

    cmd->registerIntData(ServiceCommands::IntDataBuilder("gateSeconds", true)
        .cmd("gateSeconds")
        .help("--> Duration of gating for the counter, in seconds")
        .vMin(1)
        .vMax(20000000)
        .ptr(&gateSeconds)
    );

    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("enabled", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .helpOn("--> Enable the counter")
        .helpOff("--> Disable the counter")
        .ptr(&isEnabled)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (isLoading) {
                isEnabled = val;
                return true;
            }
            if (val == isEnabled) {
                *msg = String("Already ") + (isEnabled ? "enabled" : "disabled");
            } else {
                if (val) {
                    enable();
                } else {
                    disable();
                }
                *msg = (isEnabled ? "Enabled" : "Disabled");
            }
            return true;
        })
    );

    cmd->registerIntData(ServiceCommands::IntDataBuilder("error", true)
        .cmd("error")
        .help("--> Current cumulative error, in ticks, since \"errorDuration\"")
        .isPersistent(false)
        .getFn([this]() { return error; })
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("errorDuration", true)
        .cmd("errorDuration")
        .help("--> Duration in seconds since start of cumulating \"error\"")
        .isPersistent(false)
        .getFn([this]() { return errorDuration; })
    );

    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("resetError", true)
        .cmdOn("resetError")
        .helpOn("--> Restart cumulating error")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            error = 0;
            errorDuration = 0;
            *msg = "Error reset";
            return true;
        })
    );

}

#endif
