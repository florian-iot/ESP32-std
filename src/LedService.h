#ifndef INCL_LED_MGR_H
#define INCL_LED_MGR_H

#include "CompilationOpts.h"

#ifdef USE_LED

#include <FastLED.h>
#include "UEvent.h"
#include "Random.h"
#include "CommandMgr.h"

class Effect {
protected:
    int controllerId;
    CRGB *leds;
    int totalLedCount;
    String description;
public:
    Effect(int controllerId, CRGB *leds, int totalLedCount, const char *description);
    virtual ~Effect();
    virtual void init(int millisPerFrame, CommandMgr *commandMgr, uint32_t frame) = 0;
    virtual void setFrameDuration(int millisPerFrame) = 0;
    virtual void onAfterLoad(int millisPerFrame) = 0;
    virtual void getOneLineStatus(String *msg) = 0;
    virtual void calc(uint32_t frame) = 0;
    virtual void printStats() = 0;
    virtual void printStats(String *msg) = 0;
};

class LedService {
private:
    UEventLoop *eventLoop;
    ServiceCommands *cmd;
    UEventLoopTimer timer;

    static const int LED_HARDWARE_COUNT = 8;
    struct Hardware {
        bool isConfigured;
        String ledChip;
        String rgbOrder;
        int ledPin;
        int count;
    };
    Hardware hardware[LED_HARDWARE_COUNT];

    int totalLedCount;
    CRGB *leds; // array of leds, of size totalLedCount, mapped sequentially to hardware

    int powerLimit;
    int intervalMillis;
    uint32_t frame;
    bool isInitialized;
    String initializationError;
    bool isEnabled;
    int brightness;
    long lastRunTm = 0;

    uint32_t avgShowDuration = 0;
    uint32_t avgTotalDuration = 0;
    uint32_t avgFrameInterval = 0;

    int controllerCount;
    Effect **controllers; // controllerCount elements

    void enable();
    void disable();
    void runOnce();

    bool loadHardwareAndControllers(UEventLoop *eventLoop, CommandMgr *commandMgr);
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr);
};


#endif
#endif
