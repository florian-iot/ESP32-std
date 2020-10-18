#include "CompilationOpts.h"

#ifdef USE_STEPPER

#include <FreeRTOS.h>
#include <freertos/task.h>
#include <HardwareSerial.h>
#include <AccelStepper.h>
#include <FastLED.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "Stepper.h"


/*

TODO:

check limits, don't go to negative,...
beeper

*/


const char *Stepper::stateNames[NUMBER_OF_STATES] = {
    "STEPPER_DISABLED",
    "ALIGN",
    "ALIGN_ERROR",
    "IDLE",
    "MOVE_TO_PRE_TARGET",
    "MOVE_TO_TARGET",
    "MOVE_BACKOFF",
    "MOVE_ERROR"
};

Stepper::Stepper()
:
    stepper(nullptr),
    stepperEventLoop("Stepper"),
    stepperPrintTimer(eventLoop, "stepperPrintTimer"),
    stepperRunTimer(nullptr, "stepperRunTimer")
{
    stepperEventCmd = stepperEventLoop.getEventType("data:StepperEventData", "data");

    // set default values, stepper not enabled

    pinDir = STEPPER_DIR_PIN;
    pinStep = STEPPER_STEP_PIN;
    pinM0 = STEPPER_M0_PIN;
    pinM1 = STEPPER_M1_PIN;
    pinM2 = STEPPER_M2_PIN;
    pinEnable = STEPPER_ENABLE_PIN;
    pinReset = STEPPER_RESET_PIN;
    pinSleep = STEPPER_SLEEP_PIN;
    pinFault = STEPPER_FAULT_PIN;
    pinStopper = STEPPER_STOPPER_PIN;
    pinStopperPowerSense = STEPPER_STOPPER_POWERSENSE_PIN;
    pinDebug = STEPPER_DEBUG_PIN;
    pinBuzzer = STEPPER_BUZZER_PIN;
    pinLed = STEPPER_LED_PIN;

    ledController = nullptr;

    setMicrostep(32, false); // don't set pins, we don't know yet which are the right pins

    maxSpeedUSteps = 50.0f * microstep;
    maxAlignSpeedUSteps = 10.0f * microstep;
    accelerationUSteps = 100.0f * microstep;
    isKeepTorque = false;
    isEnabled = false;
    isDirectionInversed = false;

    ledColorDisabled = 0x000206;
    ledColorIdle = 0x000600;
    ledColorActive = 0x080100;
    ledColorError = 0x800000;

    stopperLevel = 0;
    stopperVoltage = 0;
    faultLevel = 0;

    isAutoAlign = false;
    isUseStopper = false;
    isIgnoreHardware = false;
    maxPosition = -1;

    finalTargetPosSteps = 0;
    slackSteps = 0;
    stopperGuardSteps = 20;
    curForwardSlackUSteps = 0;
    curBackwardSlackUSteps = 0;

    state = STEPPER_DISABLED;
}

void Stepper::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->logMgr = logMgr;
    stepperEventSem = xSemaphoreCreateBinary();
    stopperVoltageTimer.init(eventLoop, nullptr);

    logger = logMgr->newLogger("stepper");

    cmd = commandMgr->getServiceCommands("stepper");

    // Note: we start disabled, only enable after loading settings
    // Note2: any change in pins disables the stepper

    auto registerPin = [this](const char *name, const char *help, int *pinVar) {
        cmd->registerIntData(ServiceCommands::IntDataBuilder(name, true)
            .cmd(name)
            .isPersistent(true)
            .help(help)
            .vMin(-1)
            .vMax(99)
            .setFn([this, pinVar, name](int val, bool isLoading, String *msg) {
                if (isLoading) {
                    *pinVar = val;
                    return true;
                }
                bool didDisable = false;
                if (isEnabled) {
                    StepperEventData data;
                    processInStepperThread(data.disable());
                    didDisable = true;
                }
                *pinVar = val;
                if (didDisable) {
                    *msg = "Disabled stepper, set "; *msg += name; *msg += " to "; *msg += val;
                } else {
                    *msg = "Stepper is already disabled, set "; *msg += name; *msg += " to "; *msg += val;
                }
                return true;
            })
            .ptr(pinVar)
        );
    };

    registerPin("pinDir", "-- Direction pin", &pinDir);
    registerPin("pinStep", "-- Step pin", &pinStep);
    registerPin("pinM0", "-- Multistep pin 0", &pinM0);
    registerPin("pinM1", "-- Multistep pin 1", &pinM1);
    registerPin("pinM2", "-- Multistep pin 2", &pinM2);
    registerPin("pinEnable", "-- Enable pin", &pinEnable);
    registerPin("pinReset", "-- Reset pin", &pinReset);
    registerPin("pinSleep", "-- Sleep pin", &pinSleep);
    registerPin("pinFault", "-- Fault pin", &pinFault);
    registerPin("pinStopper", "-- Stopper pin", &pinStopper);
    registerPin("pinStopperPowerSense", "-- Stopper power sense pin", &pinStopperPowerSense);
    registerPin("pinBuzzer", "-- Buzzer pin", &pinBuzzer);
    registerPin("pinLed", "-- Led pin, needs reboot to take in account", &pinLed);
    registerPin("pinDebug", "-- Debug pin, high when performing one step", &pinDebug);

    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("enabled", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .isPersistent(true)
        .helpOn("-- Enable the stepper")
        .helpOff("-- Disable the stepper")
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (isLoading) { // we don't want to enable while loading settings
                isEnabledFromLoad = val;
                return true;
            }
            if (val == isEnabled) {
                *msg = "Stepper is already "; *msg += (isEnabled ? "enabled" : "disabled");
                return true;
            }
            StepperEventData data;
            if (val) {
                processInStepperThread(data.enable());
            } else {
                processInStepperThread(data.disable());
            }
            *msg = "Stepper is "; *msg += (isEnabled ? "enabled" : "disabled");
            return true;
        })
        .ptr(&isEnabled)
    );

    cmd->registerIntData(ServiceCommands::IntDataBuilder("go", true)
        .cmd("go")
        .isPersistent(false)
        .help("-- Number of full steps to move from the currently set target position, negative or positive")
        .setFn([this](int val, bool isLoading, String *msg) {
            if (!isEnabled) {
                *msg = "Stepper is not enabled, command ignored";
                return true;
            }
            if (isUseStopper && finalTargetPosSteps + val < 0) {
                *msg = "Target position is negative and useStopper == true, command ignored";
                return true;
            }
            if (maxPosition != -1 && finalTargetPosSteps + val > maxPosition) {
                *msg = "Target position is greater than max position ("; *msg += maxPosition; *msg += "), command ignored";
                return true;
            }
            StepperEventData data;
            processInStepperThread(data.move(val));
            long ret = data.moveVal;

            *msg = "New target position set to: "; *msg += ret;
            return true;
        })
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("goto", true)
        .cmd("goto")
        .isPersistent(false)
        .help("-- Move to new target position")
        .setFn([this](int val, bool isLoading, String *msg) {
            if (!isEnabled) {
                *msg = "Stepper is not enabled, command ignored";
                return true;
            }
            if (isUseStopper && val < 0) {
                *msg = "Target position is negative and useStopper == true, command ignored";
                return true;
            }
            if (maxPosition != -1 && val > maxPosition) {
                *msg = "Target position is greater than max position ("; *msg += maxPosition; *msg += "), command ignored";
                return true;
            }
            StepperEventData data;
            processInStepperThread(data.moveTo(val));
            long ret = data.moveToVal;
            *msg = "New target position set to: "; *msg += ret;
            return true;
        })
    );
    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("stop", true)
        .cmdOn("stop")
        .isPersistent(false)
        .helpOn("-- Instructs the stepper to stop, using the programmed deceleration")
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (!isEnabled) {
                *msg = "Stepper is not enabled, command ignored";
                return true;
            }
            StepperEventData data;
            processInStepperThread(data.stop(false));
            long ret = data.stopVal;
            *msg = "Stopping, new target position set to: "; *msg += ret;
            return true;
        })
    );
    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("immediateStop", true)
        .cmdOn("immediateStop")
        .isPersistent(false)
        .helpOn("-- Instructs the stepper to stop immediately, without using acceleration. This may cause step skips.")
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (!isEnabled) {
                *msg = "Stepper is not enabled, command ignored";
                return true;
            }
            StepperEventData data;
            processInStepperThread(data.stop(true));
            long ret = data.stopVal;
            *msg = "Stopping, new target position set to: "; *msg += ret;
            return true;
        })
    );
    cmd->registerFloatData(ServiceCommands::FloatDataBuilder("accel", true)
        .cmd("accel")
        .isPersistent(true)
        .help("-- Sets the acceleration, in full steps per second^2")
        .setFn([this](float val, bool isLoading, String *msg) {
            if (!isEnabled || isLoading) {
                accelerationUSteps = val * microstep;
                *msg = "Acceleration set to: "; *msg += accelerationUSteps / microstep; *msg += " (stepper is disabled)";
                return true;
            }
            StepperEventData data;
            processInStepperThread(data.accel(val));
            float ret = data.accelVal;
            *msg = "Acceleration set to: "; *msg += ret; *msg += " steps/sec^2 ("; *msg += ret * microstep; *msg += " microsteps/sec^2)";
            return true;
        })
        .getFn([this]() {
            return (accelerationUSteps / microstep);
        })
    );
    cmd->registerFloatData(ServiceCommands::FloatDataBuilder("maxSpeed", true)
        .cmd("maxSpeed")
        .isPersistent(true)
        .help("-- Sets the maximum speed, in full steps per second")
        .setFn([this](float val, bool isLoading, String *msg) {
            if (!isEnabled || isLoading) {
                maxSpeedUSteps = val * microstep;
                *msg = "Max speed set to: "; *msg += (maxSpeedUSteps / microstep); *msg += " steps/sec (";
                *msg += maxSpeedUSteps; *msg += " microsteps/sec) (stepper is disabled)";
                return true;
            }
            StepperEventData data;
            processInStepperThread(data.maxSpeed(val));
            float ret = data.maxSpeedVal;
            *msg = "Max speed set to: "; *msg += ret; *msg += " steps/sec ("; *msg += ret * microstep; *msg += " microsteps/sec)";
            return true;
        })
        .getFn([this]() {
            return (maxSpeedUSteps / microstep);
        })
    );
    cmd->registerFloatData(ServiceCommands::FloatDataBuilder("maxAlignSpeed", true)
        .cmd("maxAlignSpeed")
        .isPersistent(true)
        .help("-- Sets the maximum speed during alignment, in full steps per second")
        .setFn([this](float val, bool isLoading, String *msg) {
            if (!isEnabled || isLoading) {
                maxAlignSpeedUSteps = val * microstep;
                *msg = "Max speed for alignment set to: "; *msg += (maxAlignSpeedUSteps / microstep); *msg += " steps/sec (";
                *msg += maxAlignSpeedUSteps; *msg += " microsteps/sec) (stepper is disabled)";
                return true;
            }
            StepperEventData data;
            processInStepperThread(data.maxAlignSpeed(val));
            float ret = data.maxAlignSpeedVal;
            *msg = "Max speed during alignment set to: "; *msg += ret; *msg += " steps/sec ("; *msg += ret * microstep; *msg += " microsteps/sec)";
            return true;
        })
        .getFn([this]() {
            return (maxAlignSpeedUSteps / microstep);
        })
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("microstepping", true)
        .cmd("microstepping")
        .isPersistent(true)
        .help("-- Sets the microstepping to one of 1, 2, 4, 8, 16 or 32")
        .setFn([this](int val, bool isLoading, String *msg) {
            if (val != 1 && val != 2 && val != 4 && val != 8 && val != 16 && val != 32) {
                *msg = "Invalid microstepping value of "; *msg += val;
                return true;
            }
            int oldMicrostep = microstep;
            if (!isEnabled || isLoading) {
                microstep = val;
                maxSpeedUSteps = maxSpeedUSteps / oldMicrostep * microstep;
                maxAlignSpeedUSteps = maxAlignSpeedUSteps / oldMicrostep * microstep;
                accelerationUSteps = accelerationUSteps / oldMicrostep * microstep;
                *msg = "Microstepping set to: "; *msg += (maxSpeedUSteps / microstep); *msg += " (stepper is disabled)";
                return true;
            }
            StepperEventData data;
            processInStepperThread(data.microstepping(val));
            int ret = data.microsteppingVal;
            *msg = "Microstepping set to "; *msg += ret;
            return true;
        })
        .ptr(&microstep)
    );
    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("keepTorque", true)
        .cmd("keepTorque")
        .isPersistent(true)
        .help("-- keepTorque on|off -- If set, the stepper will keep the torque while idle")
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (!isEnabled || isLoading) {
                isKeepTorque = val;
                *msg = "Set keepTorque to "; *msg += (isKeepTorque ? "on" : "off");
                return true;
            }
            StepperEventData data;
            processInStepperThread(data.keepTorque(val));
            long ret = data.keepTorqueVal;
            *msg = "Set keepTorque to "; *msg += (ret ? "on" : "off");
            return true;
        })
        .ptr(&isKeepTorque)
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("ledColorDisabled", true)
        .cmd("ledColorDisabled")
        .help("-- Led color when disabled (0xRRGGBB)")
        .isShowAsHex()
        .vMax(0xFFFFFF)
        .vMin(0)
        .ptr(&ledColorDisabled)
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("ledColorIdle", true)
        .cmd("ledColorIdle")
        .help("-- Led color when idle (0xRRGGBB)")
        .isShowAsHex()
        .vMax(0xFFFFFF)
        .vMin(0)
        .ptr(&ledColorIdle)
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("ledColorActive", true)
        .cmd("ledColorActive")
        .help("-- Led color when active (running, hodling torque) (0xRRGGBB)")
        .isShowAsHex()
        .vMax(0xFFFFFF)
        .vMin(0)
        .ptr(&ledColorActive)
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("ledColorError", true)
        .cmd("ledColorError")
        .help("-- Led color in error condition (0xRRGGBB)")
        .isShowAsHex()
        .vMax(0xFFFFFF)
        .vMin(0)
        .ptr(&ledColorError)
    );
    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("directionInversed", true)
        .cmd("directionInversed")
        .help("-- If set, moving direction is inversed. Disables the stepper when set.")
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (isLoading) {
                isDirectionInversed = val;
                return true;
            }
            bool didDisable = false;
            if (isEnabled) {
                StepperEventData data;
                processInStepperThread(data.disable());
                didDisable = true;
            }
            isDirectionInversed = val;
            if (didDisable) {
                *msg = "Disabled stepper, set directionInversed to "; *msg += val;
            } else {
                *msg = "Stepper is already disabled, set directionInversed to "; *msg += val;
            }
            return true;
        })
        .ptr(&isDirectionInversed)
    );
    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("autoAlign", true)
        .cmd("autoAlign")
        .help("-- If set, alignment is performed when the stepper is enabled")
        .ptr(&isAutoAlign)
    );
    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("useStopper", true)
        .cmd("useStopper")
        .help("-- If set, the stopper is used: an error state is entered if the stopper is encountered")
        .ptr(&isUseStopper)
    );
    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("ignoreHardware", true)
        .cmd("ignoreHardware")
        .help("-- If set, hardware sensors indicating failure are ignored, and we can run with just the microcontroller, without other hardware")
        .ptr(&isIgnoreHardware)
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("maxPosition", true)
        .cmd("maxPosition")
        .help("-- Maximum position, -1 for no limit. Minimum position is 0 if the stopper is used.")
        .vMin(-1)
        .ptr(&maxPosition)
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("slack", true)
        .cmd("slack")
        .help("-- Number of steps that must be performed after changing directions before the load shaft moves. Disables the stepper.")
        .vMin(0)
        .getFn([this]() {
            return (int)slackSteps;
        })
        .setFn([this](int val, bool isLoading, String *msg) {
            if (isLoading) {
                slackSteps = val;
                return true;
            }
            bool didDisable = false;
            if (isEnabled) {
                StepperEventData data;
                processInStepperThread(data.disable());
                didDisable = true;
            }
            slackSteps = val;
            if (didDisable) {
                *msg = "Disabled stepper, set slackSteps to "; *msg += val;
            } else {
                *msg = "Stepper is already disabled, set slackSteps to "; *msg += val;
            }
            return true;
        })
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("stopperGuard", true)
        .cmd("stopperGuard")
        .help("-- Number of steps to prevent from moving over the stopper, even when going to position 0. Disables the stepper.")
        .vMin(0)
        .getFn([this]() {
            return (int)stopperGuardSteps;
        })
        .setFn([this](int val, bool isLoading, String *msg) {
            if (isLoading) {
                stopperGuardSteps = val;
                return true;
            }
            bool didDisable = false;
            if (isEnabled) {
                StepperEventData data;
                processInStepperThread(data.disable());
                didDisable = true;
            }
            stopperGuardSteps = val;
            if (didDisable) {
                *msg = "Disabled stepper, set stopperGuard to "; *msg += val;
            } else {
                *msg = "Stepper is already disabled, set stopperGuard to "; *msg += val;
            }
            return true;
        })
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("position", true)
        .cmd("position")
        .isPersistent(false)
        .help("-- Target position in full steps (use \"go\" or \"goto\" to move to a position)")
        .getFn([this]() {
            if (!isEnabled) {
                return -1;
            }
            return (int)finalTargetPosSteps;
        })
    );


    cmd->registerIntData(ServiceCommands::IntDataBuilder("beep", true)
        .cmd("beep")
        .isPersistent(false)
        .help("-- Beep <volume, 1 .. 100 or 0 to turn off>")
        .setFn([this](int val, bool isLoading, String *msg) {
            if (pinBuzzer != -1) {
                if (val > 0) {
                    ledcSetup(0, 2000, 8);
                    ledcAttachPin(pinBuzzer, 0);
                    ledcWriteTone(0, 3520);
                    ledcWrite(0, val);
                } else {
                    ledcWrite(0, 0);
                }
            }
            if (val > 0) {
                *msg = "Beeper "; *msg += val;
            } else {
                *msg = "Beeper off";
            }
            return true;
        })
    );

    cmd->onBeforeLoad([this](String *msg) {
        if (isEnabled) {
            StepperEventData data;
            processInStepperThread(data.disable());
        }
        isEnabledFromLoad = false;
        return true;
    });
    cmd->onAfterLoad([this](String *msg) {
        StepperEventData data;
        if (isEnabledFromLoad) {
            processInStepperThread(data.enable());
        } else {
            processInStepperThread(data.disable());
        }
        return true;
    });

    cmd->onAfterStatus([this](String *msg) {
        *msg += "Current state: "; *msg += stateNames[state]; *msg += "\n";
        if (isEnabled) {

            StepperEventData data;
            processInStepperThread(data.getState());
            int ustep = microstep;
            float accelUSteps = accelerationUSteps;
            bool outputEnabled = isKeepTorque || data.state.isRunning;
            int msShift = microstepShift;
            int msMask = microstepMask;
            String &m = *msg;
            m += "Execution status:\n";
            m += "    Running:                 "; m += data.state.isRunning ? "true" : "false"; m += "\n";
            m += "    Microstepping:           "; m += ustep; m += "\n";
            m += "    Final target position:   "; m += finalTargetPosSteps; m += " steps\n";
            m += "    Current target position: ";
                    m += (data.state.targetPos >> msShift); m += " steps ("; m += data.state.targetPos; m += " microsteps)\n";
            m += "    Current position:        ";
                m += (data.state.currentPos >> msShift); m += ":"; m += (data.state.currentPos & msMask); m += " steps (";
                m += data.state.currentPos; m += " microsteps)\n";
            m += "    Distance to go:          ";
                    m += (data.state.distanceToGo >> msShift); m += ":"; m += (data.state.distanceToGo & msMask); m += " steps (";
                    m += data.state.distanceToGo; m += " microsteps)\n";
            m += "    Current speed:           ";
                    m += (data.state.speedUSteps / microstep); m += " steps/sec (";
                    m += data.state.speedUSteps; m += " microsteps/sec)\n";
            m += "    Slack:                   ";
                    m += curBackwardSlackUSteps; m += " <|> "; m += curForwardSlackUSteps; m += " microsteps\n";
            m += "    Max speed:               ";
                    m += maxSpeedUSteps / microstep; m += " steps/sec (";
                    m += maxSpeedUSteps; m += " microsteps/sec, ";
                    m += (long)((10000000.0 + maxSpeedUSteps - 1) / maxSpeedUSteps); m += " micros/microstep)"; m += "\n";
            m += "    Acceleration:            "; m += accelUSteps / microstep; m += " microsteps/sec^2 ("; m += accelUSteps; m += " microsteps/sec^2)\n";
            m += "    Outputs should be:       "; m += (outputEnabled ? "enabled\n" : "disabled\n");
            m += "    Stopper level:           "; m += stopperLevel; m += ", voltage: "; m += stopperVoltage; m += "\n";
            m += "    Fault level:             "; m += faultLevel; m += "\n";
        } else {
            *msg += "    Last stopper level:      "; *msg += stopperLevel; *msg += ", voltage: "; *msg += stopperVoltage; *msg += "\n";
            *msg += "    Last fault level:        "; *msg += faultLevel; *msg += "\n";
            *msg += "Stepper is disabled";
        }

        return true;
    });

    stepperEventLoop.onEvent(stepperEventCmd, [this](UEvent *event) {
        StepperEventData *data = reinterpret_cast<StepperEventData *>(const_cast<void *>(event->dataPtr));
        switch (data->action) {
        case StepperEventData::Action::MOVE:
            if (state == State::ALIGN) { // don't move if we're still aligning
                // don't move, just return
                data->moveVal = (stepper->currentPosition() >> microstepShift);
            } else {
                data->moveVal = doMoveTo((finalTargetPosSteps >> microstepShift) + data->moveVal,
                    (slackSteps > 0 ? State::MOVE_TO_PRE_TARGET : State::MOVE_TO_TARGET));
            }
            break;
        case StepperEventData::Action::MOVETO:
            if (state == State::ALIGN) { // don't move if we're still aligning
                data->moveVal = (stepper->currentPosition() >> microstepShift);
            } else {
                data->moveToVal = doMoveTo(data->moveToVal, (slackSteps > 0 ? State::MOVE_TO_PRE_TARGET : State::MOVE_TO_TARGET));
            }
            break;
        case StepperEventData::Action::STOP:
            {
                long stoppingPosition;
                if (data->stopVal) { // immediate stop
                    stoppingPosition = stepper->currentPosition();
                } else {
                    stoppingPosition = calcStoppingPosition();
                }
                stepper->moveTo(stoppingPosition);
                setNextStep();
                data->stopVal = (stepper->targetPosition() << microstepShift);
            }
            break;
        case StepperEventData::Action::MAX_SPEED:
            {
                maxSpeedUSteps = data->maxSpeedVal * microstep;
                if (state != State::ALIGN) { // don't change the current speed if we're during alignment
                    stepper->setMaxSpeed(maxSpeedUSteps);
                    data->maxSpeedVal = stepper->maxSpeed() / microstep;
                    setNextStep();
                } else {
                    data->maxSpeedVal = maxSpeedUSteps / microstep;
                }
            }
            break;
        case StepperEventData::Action::MAX_ALIGN_SPEED:
            {
                maxAlignSpeedUSteps = data->maxAlignSpeedVal * microstep;
                if (state == State::ALIGN) { // change current speed only if we're during alignment
                    stepper->setMaxSpeed(maxAlignSpeedUSteps);
                    data->maxAlignSpeedVal = stepper->maxSpeed() / microstep;
                    setNextStep();
                } else {
                    data->maxAlignSpeedVal = maxAlignSpeedUSteps / microstep;
                }
            }
            break;
        case StepperEventData::Action::ACCEL:
            {
                accelerationUSteps = (data->accelVal * microstep);
                stepper->setAcceleration(accelerationUSteps);
                data->accelVal = accelerationUSteps / microstep;
                setNextStep();
            }
            break;
        case StepperEventData::Action::MICROSTEPPING:
            {
                int oldMicrostep = microstep;

                setMicrostep(data->microsteppingVal, true);

                // we must update speed, acceleration, target, maxSpeed
                float speed = stepper->speed();
                stepper->setSpeed(speed * microstep / oldMicrostep);
                accelerationUSteps = accelerationUSteps * microstep / oldMicrostep;
                stepper->setAcceleration(accelerationUSteps);
                maxSpeedUSteps = maxSpeedUSteps * microstep / oldMicrostep;
                maxAlignSpeedUSteps = maxAlignSpeedUSteps * microstep / oldMicrostep;
                if (state == State::ALIGN) {
                    stepper->setMaxSpeed(maxAlignSpeedUSteps);
                } else {
                    stepper->setMaxSpeed(maxSpeedUSteps);
                }

                long target = stepper->targetPosition();
                long curPos = stepper->currentPosition();
                // TODO: we may lose microsteps by the following division, because we may be in the middle of a range of microsteps
                // better store this operation and do it when in a full step.
                target = curPos + (target - curPos) * microstep / oldMicrostep;
                stepper->moveTo(target);

                data->microsteppingVal = microstep;
                setNextStep();
            }
            break;
        case StepperEventData::Action::KEEP_TORQUE:
            {
                isKeepTorque = data->keepTorqueVal;
                if (!stepper->isRunning()) {
                    if (isKeepTorque) {
                        stepper->enableOutputs();
                    } else {
                        stepper->disableOutputs();
                    }
                    setState(state); // we're not changing state, but will update the led
                }
                // don't forget to stepper->disableOutputs() to low when terminating a run, if !isKeepTorque
                data->keepTorqueVal = isKeepTorque;
                setNextStep();
            }
            break;
        case StepperEventData::Action::ENABLE:
            enable();
            break;
        case StepperEventData::Action::DISABLE:
            disable();
            break;
        case StepperEventData::Action::GET_STATE:
            data->state.isRunning = stepper->isRunning();
            data->state.targetPos = stepper->targetPosition();
            data->state.currentPos = stepper->currentPosition();
            data->state.distanceToGo = stepper->distanceToGo();
            data->state.speedUSteps = stepper->speed();
            data->state.maxSpeedUSteps = stepper->maxSpeed();
            break;
        default:
            /* do nothing */
            break;
        }
        return true;
    });

    dataEventType = stepperEventLoop.getEventType("data:StepperEventData", "step");

    stepperRunTimer.setCallback([this](Esp32Timer *timer, uint64_t expectedTime) {
        if (pinDebug != -1) {
            gpio_set_level((gpio_num_t)pinDebug, 1);
        }
        UEvent ev;
        ev.eventType = dataEventType;
        ev.dataInt = expectedTime;
        while (!stepperEventLoop.queueEvent(ev, nullptr, nullptr)) {
            ; // we *must* get the event to the queue, else our run is stopped
        }
        taskYIELD();
    });

    stepperEventLoop.onEvent(dataEventType, [this](UEvent *event) {
        if (pinDebug != -1) {
            gpio_set_level((gpio_num_t)pinDebug, 0);
        }
        doStep((uint64_t)event->dataInt);
        return true;
    });

    // Initialization
    void stepperEventLoopThreadFn(void *arg); // forward declaration
    xTaskCreatePinnedToCore(stepperEventLoopThreadFn, "StepperEvtLoop", 4096, this,
            uxTaskPriorityGet(nullptr) + 1, &stepperEventLoopTask, APP_CPU_NUM); // shall we keep this pinned to APP_CPU?

    // load settings
    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg); // this may already enable the stepper
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    } // else we don't enable, default state is not enabled

    initLed();

    stepperPrintTimer.setCallback([this](Esp32Timer *timer, uint64_t expectedTime) { runPrintOnce(); });
    stepperPrintTimer.setIntervalMicros(1000000);

}

void Stepper::initLed()
{
    if (ledController == nullptr && pinLed != -1 && FastLED.count() == 0) { // leds can be initialized only once, don't initialize twice (but reboot)
        digitalWrite(pinLed, 0);
        pinMode(pinLed, OUTPUT);
        digitalWrite(pinLed, 0);
        switch (pinLed) {
            case STEPPER_LED_PIN_1: ledController = &FastLED.addLeds<PL9823, STEPPER_LED_PIN_1>(leds, 2); break;
            case STEPPER_LED_PIN_2: ledController = &FastLED.addLeds<PL9823, STEPPER_LED_PIN_2>(leds, 2); break;
            case STEPPER_LED_PIN_3: ledController = &FastLED.addLeds<PL9823, STEPPER_LED_PIN_3>(leds, 2); break;
            case STEPPER_LED_PIN_4: ledController = &FastLED.addLeds<PL9823, STEPPER_LED_PIN_4>(leds, 2); break;
            case STEPPER_LED_PIN_5: ledController = &FastLED.addLeds<PL9823, STEPPER_LED_PIN_5>(leds, 2); break;
            case STEPPER_LED_PIN_6: ledController = &FastLED.addLeds<PL9823, STEPPER_LED_PIN_6>(leds, 2); break;
            case STEPPER_LED_PIN_7: ledController = &FastLED.addLeds<PL9823, STEPPER_LED_PIN_7>(leds, 2); break;
            case STEPPER_LED_PIN_8: ledController = &FastLED.addLeds<PL9823, STEPPER_LED_PIN_8>(leds, 2); break;
            case STEPPER_LED_PIN_9: ledController = &FastLED.addLeds<PL9823, STEPPER_LED_PIN_9>(leds, 2); break;
            case STEPPER_LED_PIN_10: ledController = &FastLED.addLeds<PL9823, STEPPER_LED_PIN_10>(leds, 2); break;
            default: /* do not add leds */ ledController = nullptr; break;
        }
        if (ledController != nullptr) {
            ledController->clearLedData();
            leds[0] = CRGB(ledColorDisabled);
            leds[1] = leds[0];
            FastLED.show();
        }
    }
}

void Stepper::processInStepperThread(StepperEventData &data)
{
    UEvent ev;
    ev.eventType = stepperEventCmd;
    ev.dataPtr = &data;

    stepperEventLoop.queueEvent(ev, nullptr, stepperEventSem);
    xSemaphoreTake(stepperEventSem, portMAX_DELAY); // wait for the event to be processed
}

void Stepper::setMicrostep(int res, bool doSetPins)
{
    microstep = res;
    switch (res) {
        case 1: microstepShift = 0x0; break;
        case 2: microstepShift = 0x1; break;
        case 4: microstepShift = 0x2; break;
        case 8: microstepShift = 0x3; break;
        case 16: microstepShift = 0x4; break;
        case 32: microstepShift = 0x5; break;
        default: microstepShift = 0x5; microstep = 32; break; // default to smallest microstep
    }
    microstepMask = microstep - 1;

    if (doSetPins) {
        int valM0 = microstepShift & 0x01 ? HIGH : LOW;
        int valM1 = microstepShift & 0x02 ? HIGH : LOW;
        int valM2 = microstepShift & 0x04 ? HIGH : LOW;
        if (pinM0 != -1) {
            digitalWrite(pinM0, valM0);
        }
        if (pinM1 != -1) {
            digitalWrite(pinM1, valM1);
        }
        if (pinM2 != -1) {
            digitalWrite(pinM2, valM2);
        }
    }
}

void Stepper::enable()
{
    logger->info("Enabling");

    if (stepper != nullptr) {
        stepper->disableOutputs();
        delete stepper;
        stepper = nullptr;
    }
    digitalWrite(pinDir, LOW);
    pinMode(pinDir, OUTPUT);
    digitalWrite(pinStep, LOW);
    pinMode(pinStep, OUTPUT);
    if (pinM0 != -1) {
        digitalWrite(pinM0, 0);
        pinMode(pinM0, OUTPUT);
    }
    if (pinM1 != -1) {
        digitalWrite(pinM1, 0);
        pinMode(pinM1, OUTPUT);
    }
    if (pinM2 != -1) {
        digitalWrite(pinM2, 0);
        pinMode(pinM2, OUTPUT);
    }
    if (pinReset != -1) {
        digitalWrite(pinReset, HIGH);
        pinMode(pinReset, OUTPUT);
    }
    if (pinSleep != -1) {
        digitalWrite(pinSleep, HIGH);
        pinMode(pinSleep, OUTPUT);
    }
    if (pinFault != -1) {
        pinMode(pinFault, INPUT_PULLDOWN);
        faultLevel = digitalRead(pinFault);
    }
    if (pinDebug != -1) {
        digitalWrite(pinDebug, 0);
        pinMode(pinDebug, OUTPUT);
    }
    setMicrostep(microstep, true);
    stepper = new AccelStepper(AccelStepper::DRIVER, pinStep, pinDir, false);
    if (pinEnable != -1) {
        stepper->setEnablePin(pinEnable);
        stepper->setPinsInverted(isDirectionInversed, false, true); // our enable pin is inverted
    }
    if (isKeepTorque) {
        stepper->enableOutputs();
    } else {
        stepper->disableOutputs();
    }
    stepper->setAcceleration(accelerationUSteps);
    stepper->setMaxSpeed(maxSpeedUSteps);
    stepper->setMinPulseWidth(4); // datasheet indicates minimum 1.9 us, we set it slightly more -- see datasheet

    if (pinDebug != -1) {
        digitalWrite(pinDebug, 0);
        pinMode(pinDebug, OUTPUT);
    }

    isEnabled = true;

    lastPos = 0;
    avgDelay = 0;
    maxDelay = 0;
    minStepInterval = 0;

    initLed();

    if (pinStopper != -1) {
        pinMode(pinStopper, INPUT_PULLUP);
        // do a first read right now, but wait for pull-up to establish
        delay(1);
        stopperLevel = digitalRead(pinStopper);
    }
    if (pinStopperPowerSense != -1) {
        adcAttachPin(pinStopperPowerSense);
        stopperVoltage = analogRead(pinStopperPowerSense);
        stopperVoltageTimer.setCallback([this](UEventLoopTimer *timer) {
            stopperVoltage = analogRead(pinStopperPowerSense);
        });
        stopperVoltageTimer.setInterval(333);
        // do a first read right now
        stopperVoltage = analogRead(pinStopperPowerSense);
    }

    // a first reading of stopper voltage is performed above, it's needed for below
    curForwardSlackUSteps = slackSteps * microstep; // this will also be set after alignment finishes, if we auto-align
    curBackwardSlackUSteps = slackSteps * microstep;
    if (isAutoAlign && !isIgnoreHardware) {
        if (!isStopperVoltageNominal()) {
            stepper->disableOutputs();
            setState(State::ALIGN_ERROR);
        } else {
            stepper->setMaxSpeed(maxAlignSpeedUSteps);
            doMoveTo(maxPosition != -1 ? -maxPosition : -200 * 1000 /* a big enough number of revolutions */, State::ALIGN);
        }
    } else {
        setState(State::IDLE);
    }

}

void Stepper::disable()
{
    logger->info("Disabling");

    if (stepper != nullptr) {
        stepper->disableOutputs();
        delete stepper;
        stepper = nullptr;
    }
    if (pinDebug != -1) {
        pinMode(pinDebug, INPUT);
    }
    if (pinStopper != -1) {
        pinMode(pinStopper, INPUT);
    }
    if (pinStopperPowerSense != -1) {
        pinMode(pinStopperPowerSense, INPUT);
        stopperVoltageTimer.cancelInterval();
    }

    isEnabled = false;
    setState(State::STEPPER_DISABLED);
}

bool Stepper::isStopperVoltageNominal()
{
    return (pinStopperPowerSense == -1) || isIgnoreHardware || (stopperVoltage > 1000 && stopperVoltage < 2000);
}

/** Move to position, in full steps */
long Stepper::doMoveTo(long posStep, State newState)
{
    logger->debug("doMoveTo({}, {})", (int64_t)posStep, stateNames[newState]);

    if (!stepper->isRunning() && !isKeepTorque) {
        stepper->enableOutputs();
        delayMicroseconds(10); // give a bit of time to enable
    }

    if (newState == State::MOVE_TO_PRE_TARGET) {
        finalTargetPosSteps = posStep;

        long finalTargetUSteps = (finalTargetPosSteps << microstepShift);
        long slackUSteps = (slackSteps << microstepShift);
        long curPos = stepper->currentPosition();
        float curSpeed = stepper->speed();
        long stopPos = calcStoppingPosition();

        // At target we need to have forwardSlack == 0.
        // When we go forward, we decrease forwardSlack up to 0, increase backwardSlack up to slack.
        // When we go backward, we increase forwardSlack up to slack, we decrease backwardSlack up to 0.

        // if going forward
        if (curSpeed >= 0) {
            // if target is forward more than forwardSlack, and if we can stop at target
            // then [*] go to target
            // else calculate position for stopping as soon as possible, forward and backward slack there, and calculate from that stopped position
            //   we'll stop at newPosition = currentPosition + stopDistance,
            //   here we know that newPosition + newForwardSlack > target
            //   so we need to go backwards to [*] targetPosition - slack, then [*] go forward to target

            if (finalTargetUSteps >= curPos + curForwardSlackUSteps && finalTargetUSteps >= stopPos) {
                setState(State::MOVE_TO_TARGET);
                stepper->moveTo(finalTargetUSteps);
            } else {
                setState(State::MOVE_TO_PRE_TARGET);
                stepper->moveTo(finalTargetUSteps - slackUSteps);
            }
        } else {
            // if going backward, we must stop at a point where the position + forwardSlack at that point < target position, then
            // move forward.

            // if current position + forwardSlack < target position
            //   then we can stop as soon as possible, because whenever we stop, position + forwardSlack there < target, then [*] go to target
            // else (current position + forwardSlack > targetPosition)
            //   we must go backwards at at least targetPosition - slack, then move forwards
            //   if we can stop before targetPosition - slack, [*] go at targetPosition - slack and [*] go forward to target
            //   else stop as soon as possible and [*] go forward to target

            if (finalTargetUSteps >= curPos + curForwardSlackUSteps) {
                setState(State::MOVE_TO_TARGET);
                stepper->moveTo(finalTargetUSteps);
            } else {
                if (stopPos > finalTargetUSteps - slackUSteps) {
                    setState(State::MOVE_TO_PRE_TARGET);
                    stepper->moveTo(finalTargetUSteps - slackUSteps);
                } else {
                    setState(State::MOVE_TO_TARGET);
                    stepper->moveTo(finalTargetUSteps);
                }
            }
        }
    } else { // here we go directly to target
        setState(newState);
        finalTargetPosSteps = posStep;
        stepper->moveTo(finalTargetPosSteps << microstepShift);
    }

    setNextStep();
    return (stepper->targetPosition() >> microstepShift);
}

// returns nearest stopping position in microsteps
long Stepper::calcStoppingPosition()
{
    // Find out the target for stopping, given the current speed and acceleration. Round to
    // full steps. (Stepper::stop() does not round to full steps.)
    float speed = stepper->speed();
    if (speed != 0.0) {
        // from Stepper code
        long stepsToStop = (long)((speed * speed) / (2.0 * accelerationUSteps) + 1); // Equation 16 (+integer rounding)
        if (speed < 0) {
            stepsToStop = -stepsToStop;
        }
        long targetPos = stepper->currentPosition() + stepsToStop;
        // round the target pos
        if (speed < 0) { // round down
            targetPos = targetPos & ~((unsigned long)microstepMask);
        } else { // round up
            targetPos = (targetPos + microstep - 1) & ~((unsigned long)microstepMask);
        }
        return targetPos;
    } else {
        return stepper->currentPosition();
    }
}

void stepperEventLoopThreadFn(void *arg)
{
    Stepper *thisStepper = (Stepper*)arg;
    thisStepper->initStepperLoop();
    thisStepper->stepperEventLoop.run();
}

void Stepper::initStepperLoop()
{
    stepperRunTimer.setTimeoutMicros(0); // run now
    // we start disabled. Loading can enable (as can a command), in which case a data
    // event is sent to the stepping event queue, and that will perform the enabling
}

void Stepper::setNextStep()
{
    unsigned long tm = micros();
    unsigned long next = stepper->getNextStepTimeMicros();
    if (next == 0) {
        logger->debug("Stopping from setNextStep()");
        // we're not stepping
        avgDelay = 0;
        minStepInterval = 0;
        if (!isKeepTorque) {
            stepper->disableOutputs(); // TODO may want to set a timer to do this 1 sec after stopping (and not starting again)
        }
        setState(State::IDLE);
        return;
    }
    // minStepInterval will be negative if we missed a deadline
    if (minStepInterval == 0 || (long)(next - tm) < minStepInterval) {
        minStepInterval = (long)(next - tm);
    }
    if ((long)(next - tm) > 0) {
        // we need to step in the future
// logger->trace("On setNextStep(), setting timeout to {} (tm={} next={})", (int64_t)((long)(next - tm)), (uint64_t)tm, (uint64_t)next);
        stepperRunTimer.setTimeoutMicros((long)(next - tm));
    } else { // step right now, we're late
// logger->trace("On setNextStep(), we're late, setting timeout to 0");
        stepperRunTimer.setTimeoutMicros(0);
    }
}

void Stepper::doStep(uint64_t expectedTime)
{
    if (!isEnabled) {
        return;
    }

    uint64_t currentTime = Esp32Timer::currentTime();

    /*************/
    long distanceToGo = stepper->distanceToGo();
    bool isRunning = stepper->run();

    if (distanceToGo > 0) { // increments and decrements of current slack depends on direction we've moved
        if (curForwardSlackUSteps > 0) {
            curForwardSlackUSteps--;
        }
        if (curBackwardSlackUSteps < (slackSteps << microstepShift)) {
            ++curBackwardSlackUSteps;
        }
    } else {
        if (curBackwardSlackUSteps > 0) {
            curBackwardSlackUSteps--;
        }
        if (curForwardSlackUSteps < (slackSteps << microstepShift)) {
            ++curForwardSlackUSteps;
        }
    }
    /*************/

    stopperLevel = digitalRead(pinStopper);
    faultLevel = digitalRead(pinFault);

    if (state == State::ALIGN) {
        if (!isStopperVoltageNominal()) {
            // there's an error with the stopper voltage
            isRunning = false; // this will stop the stepper, below
            logger->error("On align, stopper voltage is {}, not nominal, stopper error", stopperLevel);
            setState(State::ALIGN_ERROR);
        } else if (faultLevel == LOW) {
            isRunning = false; // this will stop the stepper, below
            logger->error("On align, fault level is low, driver error");
            setState(State::ALIGN_ERROR);
        } else if (maxPosition != -1 && stepper->currentPosition() <= -this->maxPosition * microstep) {
            // we didn't find the stopper
            isRunning = false; // this will stop the stepper, below
            logger->error("On align, current position {} reached mex position {}",
                (int64_t)stepper->currentPosition(),
                this->maxPosition * this->microstep);
            setState(State::ALIGN_ERROR);
        } else if (stopperLevel == LOW) { // reached the stopper
            // we stop here, immediately (when aligning our speed is low to allow immediate stop)
            stepper->setCurrentPosition(-(this->slackSteps + this->stopperGuardSteps) * microstep); // and stops the stepper
            // we must reset the stepper driver here, to start from home position with regards to microstepping
            stepper->disableOutputs(); // reset will disable outputs, so call disableOutputs() just so that Stepper remains consistent
            digitalWrite(pinReset, LOW);
            delayMicroseconds(10); // the datasheet does not specify this time
            digitalWrite(pinReset, HIGH);
            delayMicroseconds(10);
            if (isKeepTorque) {
                stepper->enableOutputs();
            }

            curForwardSlackUSteps = (slackSteps << microstepShift); // we moved backwards, but not sure how much, assume worst
            curBackwardSlackUSteps = (slackSteps << microstepShift);

            logger->debug("On align, sucessfully reached stopper");

            if (slackSteps == 0) {
                isRunning = false; // this will stop the stepper, below
                setState(State::IDLE);
            } else {
                doMoveTo(0, State::MOVE_TO_TARGET);
            }
        } else if (!isRunning) { // we reached max position and didn't find the stopper
            isRunning = false;
            logger->error("On align, stopped without reaching stopper");
            setState(State::ALIGN_ERROR);
        }

        if (state != State::ALIGN) { // we finished with aligning, set back maxSpeed
            stepper->setMaxSpeed(maxSpeedUSteps);
        }
    } else {
        // check for error conditions
        if (!isIgnoreHardware) {
            if (!isStopperVoltageNominal()) {
                isRunning = false; // this will stop the stepper, below
                logger->error("Stopper voltage is {}, not nominal, stopper error", stopperLevel);
                setState(State::ALIGN_ERROR);
            } else if (faultLevel == LOW) {
                isRunning = false; // this will stop the stepper, below
                logger->error("Fault level is low, driver error");
                setState(State::ALIGN_ERROR);
            } else if (isUseStopper && stopperLevel == LOW && slackSteps > 0
                    && (stepper->currentPosition() > stopperGuardSteps)) {
                // while we're at positions < stopperGuardSteps (which can be 0), we may still be over the stopper, that's ok
                isRunning = false; // this will stop the stepper, below
                logger->error("Stopper level is low, reached the stopper, should never happen");
                setState(State::ALIGN_ERROR);
            }
        }
    }

    if (!isRunning && state == State::MOVE_TO_PRE_TARGET) {
        setState(State::MOVE_TO_TARGET);
        stepper->moveTo(finalTargetPosSteps << microstepShift);
        isRunning = true;
    } else if (!isRunning && state == State::MOVE_TO_TARGET && slackSteps / 2 > 0) {
        setState(State::MOVE_BACKOFF);
        stepper->moveTo((finalTargetPosSteps - (slackSteps / 2)) << microstepShift);
        isRunning = true;
    }

    if (!isRunning) {
        if (!isKeepTorque) {
            stepper->disableOutputs(); // TODO may want to set a timer to do this 1 sec after stopping (and not starting again)
        }
        if (state == State::MOVE_TO_PRE_TARGET || state == State::MOVE_TO_TARGET || state == State::MOVE_BACKOFF) {
            setState(State::IDLE);
        } // else we've set state to an error state, or idle, no need to set state again
        avgDelay = 0;
        logger->debug("Stopped");

        return;
    } else {
        setNextStep();
    }

    long delay = currentTime - expectedTime;
    avgDelay = (avgDelay == 0 ? delay : (avgDelay * 63 + delay) / 64);
    if (delay > maxDelay) {
        maxDelay = delay;
    }
}

void Stepper::setState(State newState)
{
    logger->debug("Status set from {} to {}", stateNames[state], stateNames[newState]);
    state = newState;
    switch (state) {
    case STEPPER_DISABLED:
        leds[0] = CRGB(ledColorDisabled);
        break;
    case ALIGN:
        leds[0] = CRGB(ledColorActive);
        break;
    case ALIGN_ERROR:
        leds[0] = CRGB(ledColorError);
        break;
    case IDLE:
        leds[0] = CRGB(isKeepTorque ? ledColorActive : ledColorIdle);
        break;
    case MOVE_TO_PRE_TARGET:
        leds[0] = CRGB(ledColorActive);
        break;
    case MOVE_TO_TARGET:
    case MOVE_BACKOFF:
        leds[0] = CRGB(ledColorActive);
        leds[0] /= 2;
        break;
    case MOVE_ERROR:
        leds[0] = CRGB(ledColorError);
        break;
    case NUMBER_OF_STATES: // this is not a state, but just to remove warning
        break;
    }
    if (ledController != nullptr) {
        leds[1] = leds[0];
        FastLED.show();
    }
}

void Stepper::runPrintOnce()
{
    if (logger->isTrace() && isEnabled && stepper != nullptr && stepper->isRunning()) {
        String str;

        long curPos = stepper->currentPosition();

        str += "\tPos\tTarget\tSpeed\tTo go\tDelta\tMax delay\tAvg delay\tMin interval us\tSlack\n";
        str += "\t"; str += curPos;
        str += "\t"; str += stepper->targetPosition();
        str += "\t"; str += stepper->speed();
        str += "\t"; str += stepper->distanceToGo();
        str += "\t"; str += curPos - lastPos;
        str += "\t"; str += maxDelay;
        str += "\t"; str += avgDelay;
        str += "\t"; str += minStepInterval; str += (minStepInterval < 0 ? " (missed deadline)" : "");
        str += "\t"; str += curBackwardSlackUSteps; str += " <|> "; str += curForwardSlackUSteps;

        logger->trace("{}", LogValue(str.c_str(), LogValue::StrAction::DO_COPY));

        minStepInterval = 0;
        avgDelay = 0;
        maxDelay = 0;
        lastPos = curPos;

    }

}

#endif // USE_STEPPER
