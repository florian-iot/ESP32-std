#include <CompilationOpts.h>

#ifdef USE_BEEPER
#ifndef INCL_BEEPER_H
#define INCL_BEEPER_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <driver/ledc.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "SystemService.h"

#define MAX_TONES 5

class BuzzerPwm {
    ledc_channel_config_t ledcChannel;
    bool isInverted;
    unsigned int lastFadeEndTm;
public:
    bool init(int pin, bool isInverted, ledc_timer_t timer, ledc_channel_t channel);
    void setFreq(uint16_t freq);
    // duty is 0 .. 1000
    void setDuty(uint32_t duty, unsigned fadeMillis);
};

class BeeperService {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr, SystemService *systemService);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;

    SysPin pin;
    bool isInverted;
    bool isEnabled;

    BuzzerPwm buzzer;
    UEventLoopTimer timer;

    Dfa dfa;

    Dfa::State B_IDLE = dfa.nextState("B_IDLE");
    Dfa::State B_ACTIVE_TONE = dfa.nextState("B_ACTIVE_TONE");
    Dfa::State B_ACTIVE_SILENCE = dfa.nextState("B_ACTIVE_SILENCE");
    Dfa::State B_STOP_START = dfa.nextState("B_STOP_START");

    Dfa::Input START = dfa.nextInput("START");

    struct {
        uint8_t octave;
        uint8_t semitone;
        int millis;
        int pauseMillis; // always 0 for the last tone
    } tones[MAX_TONES];
    int toneCount;

    void initCommands(ServiceCommands *cmd);
    void initDfa();

    int dfaToneIndex;

    void setTone(int octave, int semitone);
    void stopTone();

public:
    void beep(
        int octave1, int semitone1, int millis1, int pauseMillis1 = 0,
        int octave2 = 0, int semitone2 = 0, int millis2 = 0, int pauseMillis2 = 0,
        int octave3 = 0, int semitone3 = 0, int millis3 = 0, int pauseMillis3 = 0,
        int octave4 = 0, int semitone4 = 0, int millis4 = 0, int pauseMillis4 = 0,
        int octave5 = 0, int semitone5 = 0, int millis5 = 0);
};

#endif
#endif