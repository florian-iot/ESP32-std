#ifndef INCL_LED_SPARK_EFFECT_H
#define INCL_LED_SPARK_EFFECT_H

#include "CompilationOpts.h"

#ifdef USE_LED

#include <FastLED.h>
#include "UEvent.h"
#include "Random.h"
#include "CommandMgr.h"
#include "LedService.h"

class EffectSparks: public Effect {
    friend class LedService;
private:

    bool isEnabled;
    ServiceCommands *cmd;
    int ledStart;
    int ledCount;

    class HueParams {
        friend class EffectSparks;
        friend class LedService;
        // hue1: red going to orange, hue2: yellow going to yellow-orange, 10% of the first, 90% of the second
        uint8_t hue1 = 0;
        uint8_t sat1 = 255;
        uint8_t hue1HighDelta = +30;
        uint8_t hue2 = 70;
        uint8_t sat2 = 255;
        uint8_t hue2HighDelta = -20;
        uint8_t hueJitter = 30;
        uint8_t hue1Hue2Pct = 10; // 10% hue1, rest hue2
    };
    HueParams hueParams; // current params

    int hueXScale = 1 * 256; // multiplied by 256
    int hueTScale = 1 * 256; // multiplied by 256

    uint8_t valMin = 0;
    uint8_t valMax = 255;
    uint8_t valIdle = 0;

    /**
     * Average leds that are sparking at a given moment.
     * Value between 0 and 100.
     */
    int sparkDensityPct = 20;
    int sparkDurationMillis = 500;
    int sparkAttackTimePct = 20;
    int sparkAttackEasing = 0;
    int sparkDecayTimePct = 80;
//    int sparkDecayEasing = 0;
    double sparkGamma = 2.0;

    int lastLedCount;
    int lastSparkFrames;
    int lastNonSparkFrames;
    uint32_t lastFrame;

    int millisPerFrame;
    int nonSparkFrames;
    int sparkFrames;
    int attackFrames;
    uint8_t *attackValPerFrame;
    int decayFrames;
    uint8_t *decayValPerFrame;
    int fullValFrames;
    int expDistribTable[64];
    int expDistribMean;
    uint32_t statAvgSparkingPct;

    /**
     * Led state: time when the led went on, or time in the future when it must
     * switch on (in frames).
     */
    struct LedState {
        uint32_t lastOn;
        uint8_t baseHue; // hue of the start of spark
        int hueKind:1; // if 0 we're using hue1, if 1 we're using hue2
    };
    LedState *ledState;
    Random rnd;

    void reinitLedState(uint32_t frame);
    void calcFrameProperties();
    int getRndExp();

    void setCmd(ServiceCommands *cmd, const char *commandStr, const char *help, uint8_t *data, int min, int max);
    void setCmd(ServiceCommands *cmd, const char *commandStr, const char *help, int *data, int min, int max);

public:
    EffectSparks(int controllerId, LedMap1d *leds, int totalLedCount, int defaultLedStart, int defaultLedCount, const char *description);
    virtual ~EffectSparks();
    virtual void init(int millisPerFrame, CommandMgr *commandMgr, uint32_t frame);
    virtual void onAfterLoad();
    virtual void getOneLineStatus(String *msg);
    virtual void setFrameDuration(int millis);
    virtual void calc(uint32_t frame);
    virtual void printStats();
    virtual void printStats(String *msg);
};

#endif
#endif
