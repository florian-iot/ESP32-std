#include <CompilationOpts.h>

#ifdef USE_BEEPER

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <driver/ledc.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "BeeperService.h"
#include "LogMgr.h"


bool BuzzerPwm::init(int pin, bool isInverted, ledc_timer_t timer, ledc_channel_t channel)
{
    unsigned long tm = millis();
    lastFadeEndTm = tm;

    ledc_timer_config_t ledcTimer;
    ledcTimer.duty_resolution = LEDC_TIMER_12_BIT; // allows for frequencies up to 10kHz
    ledcTimer.freq_hz = 1000;
    ledcTimer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledcTimer.timer_num = timer;
    this->isInverted = isInverted;

    esp_err_t rc = ledc_timer_config(&ledcTimer);
    if (rc != ESP_OK) {
        return false;
    }

    ledcChannel.channel = channel;
    ledcChannel.gpio_num = (gpio_num_t)pin;
    ledcChannel.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledcChannel.timer_sel = timer;
    ledcChannel.intr_type = LEDC_INTR_DISABLE;
    ledcChannel.duty = (isInverted ? 4096 : 0);
    if (ledcChannel.gpio_num > 0) {
        rc = ledc_channel_config(&ledcChannel);
        if (rc != ESP_OK) {
            return false;
        }
    }

    // Initialize fade service.
    ledc_fade_func_install(0);

    return true;
}

void BuzzerPwm::setFreq(uint16_t freq)
{
    ledc_set_freq(ledcChannel.speed_mode, ledcChannel.timer_sel, freq);
}

void BuzzerPwm::setDuty(uint32_t duty, unsigned fadeMillis)
{
    unsigned long tm = millis();
    bool stopBeforeSetting = (tm - lastFadeEndTm > 0);
    uint32_t ledcDuty = isInverted ? (1000 - duty) * 4096 / 1000 : duty * 4096 / 1000;

    if (fadeMillis > 0) {
        if (stopBeforeSetting) {
            uint32_t oldDuty = ledc_get_duty(ledcChannel.speed_mode, ledcChannel.channel);
            ledc_stop(ledcChannel.speed_mode, ledcChannel.channel, isInverted ? HIGH : LOW);
            ledc_set_duty(ledcChannel.speed_mode, ledcChannel.channel, oldDuty);
            ledc_update_duty(ledcChannel.speed_mode, ledcChannel.channel);
        }
        ledc_set_fade_with_time(ledcChannel.speed_mode, ledcChannel.channel, ledcDuty, fadeMillis);
        ledc_fade_start(ledcChannel.speed_mode, ledcChannel.channel, LEDC_FADE_NO_WAIT);
        lastFadeEndTm = tm + fadeMillis;
    } else {
        if (stopBeforeSetting) {
            ledc_stop(ledcChannel.speed_mode, ledcChannel.channel, isInverted ? HIGH : LOW);
        }
        ledc_set_duty(ledcChannel.speed_mode, ledcChannel.channel, ledcDuty);
        ledc_update_duty(ledcChannel.speed_mode, ledcChannel.channel);
        lastFadeEndTm = tm;
    }
}

void BeeperService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr, SystemService *systemService)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("beepper");

    pin = systemService->registerSysPin("beeper", "pin");
    ServiceCommands *cmd = commandMgr->getServiceCommands("beeper");
    initDfa();
    initCommands(cmd);

    // defaults
    pin.setPin(-1);
    isInverted = false;
    isEnabled = false;
    eventLoop->registerTimer(&timer);

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
    if (pin != -1) {
        digitalWrite(pin, isInverted ? HIGH : LOW);
        pinMode(pin, OUTPUT);
    }
    rc = buzzer.init(pin, isInverted, LEDC_TIMER_0, LEDC_CHANNEL_0);
    if (!rc) {
        logger->error("Buzzer failed to initialize on timer 0, channel 0");
    }
}

void BeeperService::initDfa()
{
    dfa.init(eventLoop, nullptr /* or logger, to log */, B_IDLE);

    dfa.onInput([this](Dfa *dfa, Dfa::State state, Dfa::Input input) {

        if (state.is(B_IDLE)) {
            if (input.is(START)) {
                dfaToneIndex = 0;
                return dfa->transitionTo(B_ACTIVE_TONE);
            }
        } else if (state.is(B_ACTIVE_TONE)) {
            if (input.is(Dfa::Input::ENTER_STATE)) {
                logger->trace("Starting tone[{}]", dfaToneIndex);
                setTone(tones[dfaToneIndex].octave, tones[dfaToneIndex].semitone);
                dfa->setStateTimeout(tones[dfaToneIndex].millis);
                return dfa->noTransition();
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                stopTone();
                if (dfaToneIndex == toneCount - 1) {
                    logger->trace("Terminated all tones", dfaToneIndex);
                    return dfa->transitionTo(B_IDLE);
                } else {
                    return dfa->transitionTo(B_ACTIVE_SILENCE);
                }
            } else if (input.is(START)) {
                stopTone();
                return dfa->transitionTo(B_STOP_START);
            }
        } else if (state.is(B_ACTIVE_SILENCE)) {
            if (input.is(Dfa::Input::ENTER_STATE)) {
                if (tones[dfaToneIndex].pauseMillis > 0) {
                    dfa->setStateTimeout(tones[dfaToneIndex].pauseMillis);
                    return dfa->noTransition();
                } else {
                    ++dfaToneIndex;
                    return dfa->transitionTo(B_ACTIVE_TONE);
                }
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                ++dfaToneIndex;
                return dfa->transitionTo(B_ACTIVE_TONE);
            } else if (input.is(START)) {
                return dfa->transitionTo(B_STOP_START);
            }
        } else if (state.is(B_STOP_START)) {
            if (input.is(Dfa::Input::ENTER_STATE)) {
                dfaToneIndex = 0;
                if (toneCount > 0) {
                    return dfa->transitionTo(B_ACTIVE_TONE);
                } else {
                    return dfa->transitionTo(B_IDLE);
                }
            }
        }
        return dfa->transitionError();
    });
}

void BeeperService::initCommands(ServiceCommands *cmd)
{
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&pin)
        .help("--> Set buzzer pin, set to -1 to disable; requires reboot")
    );
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("inverted", true)
        .cmd("inverted")
        .help("--> Set to true if the buzzer inverted, i.e., is on when the signal is low")
        .ptr(&isInverted)
    );
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("enabled", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .helpOn("--> Enable beeping")
        .helpOff("--> Disable beeping")
        .ptr(&isEnabled)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (!val && dfa.getState().is(B_ACTIVE_TONE, B_ACTIVE_SILENCE)) {
                stopTone();
            }
            isEnabled = val;
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("beep1", true)
        .cmd("beep1")
        .help("--> Beep for given number of milliseconds")
        .isPersistent(false)
        .vMin(10)
        .vMax(10000)
        .setFn([this](int val, bool isLoading, String *msg) {
            beep(6, 3, val);
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("beep2", true)
        .cmd("beep2")
        .help("--> Beep 2 tones for given number of milliseconds")
        .isPersistent(false)
        .vMin(10)
        .vMax(10000)
        .setFn([this](int val, bool isLoading, String *msg) {
            beep(6, 3, val / 3, 100, 6, 7, val - val / 3);
            return true;
        })
    );

}

void BeeperService::setTone(int octave, int semitone)
{
    if (isEnabled) {
        uint32_t freq = (uint32_t)(((uint32_t)55 << (octave - 1)) * pow(2, semitone / 12.0) + 0.5);
        buzzer.setFreq(freq);
        buzzer.setDuty(500, 0);
    }
}

void BeeperService::stopTone()
{
    if (isEnabled) {
        buzzer.setDuty(0, 0);
    }
}

void BeeperService::beep(
    int octave1, int semitone1, int millis1, int pauseMillis1,
    int octave2, int semitone2, int millis2, int pauseMillis2,
    int octave3, int semitone3, int millis3, int pauseMillis3,
    int octave4, int semitone4, int millis4, int pauseMillis4,
    int octave5, int semitone5, int millis5)
{
    tones[0].octave = octave1; tones[0].semitone = semitone1; tones[0].millis = millis1; tones[0].pauseMillis = pauseMillis1;
    tones[1].octave = octave2; tones[1].semitone = semitone2; tones[1].millis = millis2; tones[1].pauseMillis = pauseMillis2;
    tones[2].octave = octave3; tones[2].semitone = semitone3; tones[2].millis = millis3; tones[2].pauseMillis = pauseMillis3;
    tones[3].octave = octave4; tones[3].semitone = semitone4; tones[3].millis = millis4; tones[3].pauseMillis = pauseMillis4;
    tones[4].octave = octave5; tones[4].semitone = semitone5; tones[4].millis = millis5; tones[4].pauseMillis = 0;
    toneCount = (tones[0].millis > 0) + (tones[1].millis > 0) + (tones[2].millis > 0) + (tones[3].millis > 0) + (tones[4].millis > 0);
    logger->debug("Beeping with {} tones", toneCount);
    dfa.handleInput(START);
}


#endif
