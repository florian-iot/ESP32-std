#ifndef INCL_LED_MGR_H
#define INCL_LED_MGR_H

#include "CompilationOpts.h"

#ifdef USE_LED

#include <FastLED.h>
#include "UEvent.h"
#include "Random.h"
#include "CommandMgr.h"
#include "LogMgr.h"

#define LED_HARDWARE_COUNT 8

class LedMap1d {
friend class LedService;
    CRGB *leds;
    uint16_t *map;
public:
    CRGB *at(int pos) { return &leds[map[pos]]; }
    CRGB *atWithoutMapping(int pos) { return &leds[pos]; }
};

class Effect {
protected:
    int controllerId;
    LedMap1d *leds;
    int totalLedCount;
    String description;
public:
    Effect(int controllerId, LedMap1d *ledMap1d, int totalLedCount, const char *description);
    virtual ~Effect();
    virtual void init(int millisPerFrame, CommandMgr *commandMgr, uint32_t frame) = 0;
    virtual void setFrameDuration(int millisPerFrame) = 0;
    virtual void onAfterLoad() = 0;
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
    Logger *logger;

    int totalLedCount;
    struct Hardware {
        bool isConfigured;
        String ledChip;
        String rgbOrder;
        int ledPin;
        int count;
    };
    Hardware hardware[LED_HARDWARE_COUNT];
    LedMap1d map1d;

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
    void showMap(String *msg);
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr);
};


#endif
#endif
