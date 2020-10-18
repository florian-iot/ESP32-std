#ifndef INCL_LED_METEOR_EFFECT_H
#define INCL_LED_METEOR_EFFECT_H

#include "CompilationOpts.h"

#include <FastLED.h>
#include "UEvent.h"
#include "Random.h"
#include "CommandMgr.h"
#include "LedService.h"

class InterpolatedFunction {
    // the function is defined in [0, xEnd]
    float xEnd;
    float deltaX;

    // the array values[] contains cnt elements, cnt = xEnd / deltaX + 1.
    int cnt;
    float *values;
public:
    InterpolatedFunction();
    ~InterpolatedFunction();
    void setParams(float xSize, int count);
    void setValue(int i, float y);
    float getAt(float x);
};

inline InterpolatedFunction::InterpolatedFunction()
{
    values = nullptr;
    cnt = -1;
}

inline InterpolatedFunction::~InterpolatedFunction()
{
    delete[] values;
}

struct Shape {
    // shape
    float lengthLeds;
    float trailingPct;
    float trailingGamma;
    float leadingPct;
    float leadingGamma;

    int valMin;
    int valMax;

    Shape() {
        lengthLeds = 1;
        trailingPct = 70.0;
        trailingGamma = 1.0;
        leadingPct = 10.0;
        leadingGamma = 1;

        valMin = 0;
        valMax = 255;
    }

    void paramsCalc();
    float shapeAt(float x);

private:
    float xTrailing;
    float xLeading;
    InterpolatedFunction trailing;
    InterpolatedFunction leading;
};


struct ColorParams {

    // hue, saturation for one run
    int saturation;
    int hueStart;
    bool hueStartRandom;
    int hueStartSeqChange;
    int whiteAsRandomHuePct;

    float hueShiftInShape;
    float hueShiftInRun;

    float hueTChangeInRun;

    ColorParams() {
        saturation = 255;
        hueStart = 0;
        hueStartRandom = false;
        hueStartSeqChange = 25;
        whiteAsRandomHuePct = 2.0;
        hueShiftInShape = 40.0;
        hueShiftInRun = 0.0;
    }

};

struct TimingParams {
    // timing
    bool isExponential; // else it will be uniform
    int pauseDurationAvgMillis;
    int pauseDurationMaxMillis;

    // speed
    float speedLedsPerSec;

    TimingParams() : rndExp(1000.0) {
        isExponential = true;
        pauseDurationAvgMillis = 1000;
        pauseDurationMaxMillis = 10000;

        speedLedsPerSec = 10.0;
    }

    void paramsCalc() {
        if (pauseDurationAvgMillis <= 0) {
            pauseDurationAvgMillis = 1;
        }
        if (pauseDurationMaxMillis < pauseDurationAvgMillis) {
            pauseDurationMaxMillis = pauseDurationAvgMillis;
        }
        if (isExponential) {
            rndExp.setScale(pauseDurationAvgMillis);
        }
    }

    // returns always >0
    uint32_t getNextDelay(Random &rnd) {
        int v;
        if (isExponential) {
            v = rndExp.getRndExp(rnd);
            if (v > pauseDurationMaxMillis) {
                v = rnd.nextInt(1, pauseDurationMaxMillis - 1);
            }
            if (v == 0) {
                v = 1;
            }
        } else {
            v = rnd.nextInt(1, pauseDurationAvgMillis * 2);
            if (v > pauseDurationMaxMillis) {
                v = rnd.nextInt(1, pauseDurationMaxMillis - 1);
            }
        }
        return v;
    }

private:
    RandomExp rndExp;

};

class LedMeteorEffect: public Effect {
    friend class LedService;
private:

    bool isEnabled;
    ServiceCommands *cmd;

    int ledStart;
    int ledCount;

    Shape shape;
    ColorParams color;
    TimingParams timing;

    Random rnd;
    int millisPerFrame;

    // run-time data
    uint64_t lastFrame; // last frame seen, updated even when disabled (serves when re-enabling)
    uint64_t runCount;
    uint32_t startingFrame; // frame where start of run is planned or (if in past) has already started
    bool isRunning; // whether we're in a run, which started at "startingFrame"
    // run-time data during the run
    float curXStart; // starting position, in leds, for current run
    int startingHue;
    int startingSaturation;
    float hueShiftInRunPerFrame;
    float hueShiftInShapePerLed;

    void reinitLedState(uint32_t frame);
    void calcFrameProperties();
    int getRndExp();

    void setCmd(ServiceCommands *cmd, const char *commandStr, const char *help, uint8_t *data, int min, int max);
    void setCmd(ServiceCommands *cmd, const char *commandStr, const char *help, int *data, int min, int max);
    void setCmd(ServiceCommands *cmd, const char *commandStr, const char *help, float *data, float min, float max);
    void setCmd(ServiceCommands *cmd, const char *commandStr, const char *help, bool *data);

public:
    LedMeteorEffect(int controllerId, CRGB *leds, int totalLedCount, int defaultLedStart, int defaultLedCount, const char *description);
    virtual ~LedMeteorEffect();
    virtual void init(int millisPerFrame, CommandMgr *commandMgr, uint32_t frame);
    virtual void setFrameDuration(int millisPerFrame);
    virtual void onAfterLoad(int millisPerFrame);
    virtual void getOneLineStatus(String *msg);
    virtual void calc(uint32_t frame);
    virtual void printStats();
    virtual void printStats(String *msg);
};

#endif



