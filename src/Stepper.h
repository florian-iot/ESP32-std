#ifndef INCL_STEPPER_H
#define INCL_STEPPER_H

#include "CompilationOpts.h"
#ifdef USE_STEPPER

#include "CommandMgr.h"
#include <AccelStepper.h>
#include <FastLED.h>
#include "UEvent.h"
#include "LogMgr.h"

// Event data sent to stepperEventLoop, processed by the stepper event thread

struct StepperEventData {
    enum Action {
        NONE,
        MOVE, MOVETO, STOP, ACCEL, MAX_SPEED, MAX_ALIGN_SPEED, MICROSTEPPING, KEEP_TORQUE, ENABLE, DISABLE, GET_STATE
    };
    Action action;
    union {
        long moveVal;
        long moveToVal;
        long stopVal; // on input: true == stop immediately, false == stop using acceleration; on output, stopping step
        float accelVal;
        float maxSpeedVal;
        float maxAlignSpeedVal;
        int microsteppingVal;
        bool keepTorqueVal;
        struct {
            bool isRunning;
            long targetPos;
            long currentPos;
            long distanceToGo;
            float speedUSteps;
            float maxSpeedUSteps;
        } state;
    };

    StepperEventData() : action(Action::NONE) { }
    StepperEventData &enable() { action = Action::ENABLE; return *this; }
    StepperEventData &disable() { action = Action::DISABLE; return *this; }
    StepperEventData &move(long val) { moveVal = val; action = Action::MOVE; return *this; }
    StepperEventData &moveTo(long val) { moveToVal = val; action = Action::MOVETO; return *this; }
    StepperEventData &stop(bool isImmediate) { action = Action::STOP; stopVal = isImmediate; return *this; }
    StepperEventData &accel(float val) { accelVal = val; action = Action::ACCEL; return *this; }
    StepperEventData &maxSpeed(float val) { maxSpeedVal = val; action = Action::MAX_SPEED; return *this; }
    StepperEventData &maxAlignSpeed(float val) { maxAlignSpeedVal = val; action = Action::MAX_ALIGN_SPEED; return *this; }
    StepperEventData &microstepping(int val) { microsteppingVal = val; action = Action::MICROSTEPPING; return *this; }
    StepperEventData &keepTorque(bool val) { keepTorqueVal = val; action = Action::KEEP_TORQUE; return *this; }
    StepperEventData &getState() { action = Action::GET_STATE; return *this; }
};

class Stepper {
    friend void stepperEventLoopThreadFn(void *arg);
private:
    UEventLoop *eventLoop;
    ServiceCommands *cmd;
    LogMgr *logMgr;
    AccelStepper *stepper;

    Logger *logger;

    int pinDir;
    int pinStep;
    int pinM0;
    int pinM1;
    int pinM2;
    int pinEnable;
    int pinReset;
    int pinSleep;
    int pinFault;
    int pinStopper;
    int pinStopperPowerSense;
    int pinBuzzer;
    int pinDebug;
    int pinLed;

    float maxSpeedUSteps;
    float maxAlignSpeedUSteps; // max speed during alignment
    float accelerationUSteps;
    int microstep;
    int microstepShift;
    int microstepMask;

    long finalTargetPosSteps; // current final target (stepper's target may be the pre-target position, or target, or the retard position)
    long slackSteps;
    long stopperGuardSteps; // steps to move away from the stopper, to ensure we'll never step
                            // over the stopper even when we move to position 0 (which means we need to move to -slackSteps first)
    long curForwardSlackUSteps; // valid anytime (enabled)
    long curBackwardSlackUSteps; // valid anytime (enabled)

    bool isDirectionInversed;
    // keeps torque while idle
    bool isKeepTorque;
    bool isEnabled;
    bool isEnabledFromLoad; // tracks isEnable from load, instead of setting isEnable

    bool isAutoAlign; // when enabled, the state is set to ALIGN (when reaching there, the position is set to 0); else to IDLE
    bool isUseStopper; // if true, it's an error to step into the stopper (status goes to MOVE_ERROR)
    bool isIgnoreHardware; // if true, sensors indicating hardware failure are ignored
    int maxPosition; // -1 for no limit, the maximum allowed position. Also used to limit the move when aligning.

    int ledColorDisabled;
    int ledColorIdle;
    int ledColorActive;
    int ledColorError;

    enum State {
        STEPPER_DISABLED,
        ALIGN,
        ALIGN_ERROR,
        IDLE,
        MOVE_TO_PRE_TARGET,
        MOVE_TO_TARGET,
        MOVE_BACKOFF,
        MOVE_ERROR, // hit the stopper, found error in stopper sensing
        NUMBER_OF_STATES // this is not a state, but the total number of states
    };
    volatile State state;
    static const char *stateNames[];

    long lastPos;
    long avgDelay;
    long maxDelay;
    long minStepInterval;

    UEventLoop stepperEventLoop;
    int stepperEventCmd;
    Esp32Timer stepperPrintTimer;

    TaskHandle_t stepperEventLoopTask;
    int dataEventType;
    Esp32Timer stepperRunTimer;
    SemaphoreHandle_t stepperEventSem;

    CRGB leds[2]; // we need only 1, but we can debug with an analyzer on outgoing pin of led, and setting led 2 color identical to 1
    CLEDController *ledController;
    volatile int stopperLevel;
    int stopperVoltage;
    UEventLoopTimer stopperVoltageTimer;
    volatile int faultLevel;

public:

    Stepper();
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr);

private:
    void initLed();

    void setMicrostep(int resolution, bool doSetPins);
    void processInStepperThread(StepperEventData &data);

    void setState(State newState);
    bool isStopperVoltageNominal();

    void enable();
    void disable();
    long doMoveTo(long val, State newState);
    long calcStoppingPosition();

    // called in private thread
    void initStepperLoop();
    // called in private thread
    void doStep(uint64_t expectedTime);
    // called in private thread
    void setNextStep();

    // called in general thread
    void runPrintOnce();
};


#endif

#endif
