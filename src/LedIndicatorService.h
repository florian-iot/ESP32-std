#include <CompilationOpts.h>

#ifdef USE_LED_INDICATOR
#ifndef INCL_LED_INDICATOR_H
#define INCL_LED_INDICATOR_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "Util.h"
#include <driver/ledc.h>

#define LED_PWM_LEDCOUNT 3


class LedPwm {
    ledc_channel_config_t ledcChannel[LED_PWM_LEDCOUNT];
    bool isInverted;
    uint32_t lastFadeEndTm[LED_PWM_LEDCOUNT];

    /**
     * Format for led animation
     *
     * <animation>: <channel animation>(;<channel animation)*
     *
     * <channel annimation>:<value>(,<delay with or without fading>,<value>)*
     *
     * <channel>: c<n 0..2>
     * <value>: <n 0..255> ---> currently 8192
     * <delay without fading to next value>: d<n millis>
     * <delay with fading>: f<n millis>
     *
     * E.g.: c0:255,d500,0,d500;c1:0,f1000,255
     *
     * An animation starts all channels at once, then each channel continues independently,
     * and repeats its defined sequence.
     *
     * If an animation does not contain all available channels, only the ones included
     * in the definition are impacted. Others remain in their current definition.
     *
     */

    struct AnimationDefEntry {
        uint16_t duration;
        uint16_t type:2; // SET or FADE (we have place for 4 possibilities)
        uint16_t duty:14;
        enum Type {
            SET = 0,
            FADE = 1
        };
    };
    struct SingleChannelAnimationDef { // this is allocated
        int animId;
        int channel;
        uint16_t startingDuty;
        int entriesCount;
        AnimationDefEntry *entries;
        uint32_t totalDuration;
        SingleChannelAnimationDef() { entries = nullptr; }
        ~SingleChannelAnimationDef() { delete[] entries; }
    };
    // zero or one animation per channel
    struct AnimationDef {
        SingleChannelAnimationDef *channels[LED_PWM_LEDCOUNT];
    };
    // reference of all defined animations
    Register<AnimationDef> animationDefs;


    struct AnimationElement {
        int animId; // animation that is set for this level, -1 if none. Can't use a pointer because we're referencing into a vector
        SingleChannelAnimationDef *channels[LED_PWM_LEDCOUNT]; // simply copied from AnimationDef
        uint32_t cycles; // 0 if until removed
        struct State { // state of each of the channels (only for defined channels, where channels[j] != nullptr)
            // start time for this iteration
            long startTm; // advances always modulo channel's totalDuration
            unsigned int finishedCycles;
        } state[LED_PWM_LEDCOUNT];
    };
    std::vector<AnimationElement> animationStack;

    struct AnimHead {
        // Index in animationStack to use for this channel. Obtained by going
        // top-down in stack, and taking for each channel the first encountered entry with
        // non-empty definition for the channel
        int animHeadIdx;
        SingleChannelAnimationDef *animDef; // == animationStack[animHeadIdx].channels[channel]
        struct State { // in addition to animationStack[animHeadIdx].state[channel]
            int lastStep; // index in channels[]->entries, -1 for the very beginning
            long nextStepTm; // when is planned execution of next step
        } state;
    } animHead[LED_PWM_LEDCOUNT];

    void recalcHead();

public:
    bool init(int pin0, int pin1, int pin2, bool isInverted,
        ledc_timer_t timer = LEDC_TIMER_0,
        ledc_channel_t channel0 = LEDC_CHANNEL_0,
        ledc_channel_t channel1 = LEDC_CHANNEL_1,
        ledc_channel_t channel2 = LEDC_CHANNEL_2);

    // Duty goes from 0 to 8192.
    void setDuty(int led, uint32_t duty, unsigned fadeMillis = 0);

    // returns -1 if incorrect syntax, in which case "error" is filled
    int defineAnimation(const char *animation, String *error);
    // Would need an "undefineAnimation", leave this for later (would require handling empty spaces in "animations")
    // void undefineAnimation(int animId);
    // Concatenates animation definition to *str.
    // Returns false if the id is not a defined animation.
    bool getAnimationDef(int id, String *str);

    // returns false if animId is unknown
    bool setAnimation(int level, int animId, uint32_t cycles);
    // if animId is given, the animation at level is removed only if it corresponds to the given animId
    // returns true if the level was cleared
    bool clearAnimation(int level, int animId = -1);

    // perform any necessary transition, if any
    // return the number of milliseconds until the next transition
    // update() does not need to be called until the next transition, but it can be called (and will do nothing).
    // update() must be called in time for processing next transition.
    uint32_t update();
};


class LedIndicatorService {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr);

    // see LedPwm::defineAnimation()
    int defineAnimation(const char *animation, String *error);
    // see LedPwm::getAnimationDef()
    bool getAnimationDef(int id, String *str);
    // see LedPwm::setAnimation()
    bool setAnimation(int level, int animId, uint32_t cycles);
    // see LedPwm::clearAnimation()
    bool clearAnimation(int level, int animId = -1);

private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;

    UEventLoopTimer timer;
    bool isUpdating;

    void initCommands(ServiceCommands *cmd);
    void initDfa();

    int pin0;
    int pin1;
    int pin2;
    bool isInverted;
    int animationLevel;

    LedPwm ledPwm;

};

#endif
#endif