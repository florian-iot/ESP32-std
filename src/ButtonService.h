#include <CompilationOpts.h>

#ifdef USE_BUTTON
#ifndef INCL_BUTTON_H
#define INCL_BUTTON_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <vector>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "Util.h"
#include "SystemService.h"

class ButtonService {
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;

public:
    enum ButtonEvent {
        BUTTON_ON,
        BUTTON_OFF,
        BUTTON_CLICK,
        BUTTON_DCLICK,
        BUTTON_TCLICK,
        BUTTON_LONG_CLICK
    };

    enum ButtonState {
        BUTTON_STATE_OFF = 0x00,
        BUTTON_STATE_ON = 0x01,
        BUTTON_STATE_LONG_ON = 0x02
    };

    class SingleButtonService {
        int id;
        String serviceName; // calculated, not just const
        UEventLoop *eventLoop;
        CommandMgr *commandMgr;
        Logger *logger;

        SysPin pin;
        bool isTouch;
        bool isEnabled;
        bool isReversed;
        bool isPullUp;
        bool isPullDown;
        int glitchMillis; // pulses (low-hi-low, or hi-low-hi) of duration less than glitchMillis are ignored
        int dClickDelay;
        int tClickDelay;
        int longClickDelay;
        // We take the average reading of the sensor, and consider as "touched" state
        // any reading less than average * TOUCH_THRESHOLD / 100. Default: 75.
        int touchValueThreshold;
        int touchMillisThreshold; // only consider touch on after this time has passed below touchValueThreshold

        int touchAvg;
        int touchLongAvg;
        int touchReadMeasure;
        unsigned long touchLastAdjustTm;
        UEventLoopTimer touchTimer;
        bool touchIsOn;
        int touchIsLowCount; // count while touch is below touchValueThreshold, before turning on

        Dfa dfa;

        Dfa::State B_OFF = dfa.nextState("B_OFF");
        Dfa::State B_ON_DEBOUNCE = dfa.nextState("B_ON_DEBOUNCE");
        Dfa::State B_OFF_DEBOUNCE = dfa.nextState("B_OFF_DEBOUNCE");
        Dfa::State B_ON = dfa.nextState("B_ON");
        Dfa::State B_ON_LONG = dfa.nextState("B_ON_LONG");
        Dfa::State B_ON_2_DEBOUNCE = dfa.nextState("B_ON_2_DEBOUNCE");
        Dfa::State B_OFF_2_DEBOUNCE = dfa.nextState("B_OFF_2_DEBOUNCE");

        Dfa::Input B_TURNED_ON = dfa.nextInput("B_TURNED_ON");
        Dfa::Input B_TURNED_OFF = dfa.nextInput("B_TURNED_OFF");

        CallbackList2<bool(int id, ButtonEvent event, bool alreadyHandled), ButtonEvent> callbacks;

        int dClickHandlerCount;
        int tClickHandlerCount;
        int longClickHandlerCount;

        ButtonService::ButtonState initialState;
        ButtonService::ButtonState state;
        unsigned long onTs;
        unsigned long offTs;
        bool isClicked;

        static void IRAM_ATTR buttonIsr(void *arg);
        struct IsrCtx {
            uint8_t id;
            uint8_t pin;
            bool isReversed;
            uint32_t buttonEventType;
            volatile uint8_t irqCount;
            volatile uint16_t lostOff;
            volatile uint16_t lostOn;
            volatile uint8_t state;
            UEventLoop::IsrData isrData;
        };
        IsrCtx *isrCtx;

        void callCallbacks(ButtonEvent event);
        void initCommands(ServiceCommands *cmd);
        void initDfa();
    public:
        SingleButtonService(int id);
        void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr, SystemService *systemService);
        // returns a handle, which can be used to remove the event handler
        int onEvent(ButtonEvent event, std::function<bool(int id, ButtonEvent event, bool alreadyHandled)> fn);
        void removeOnEvent(int handle);
        ButtonState getInitialState();
        ButtonState getState();
        void enable();
        void disable();
    };

    int buttonCount; // number of buttons, configurable, we'll create all of them during initialization

    std::vector<SingleButtonService *> buttons;

    void initCommands(ServiceCommands *cmd);

public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr, SystemService *systemService);
    SingleButtonService *getButton(int id);
};

#endif
#endif
