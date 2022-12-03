#include <CompilationOpts.h>

#ifdef USE_BUTTON

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "ButtonService.h"
#include "LogMgr.h"

ButtonService::SingleButtonService::SingleButtonService(int id)
{
    this->id = id;
}

void ButtonService::SingleButtonService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr, SystemService *systemService)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;

    serviceName = "button"; serviceName.concat(id);
    pin = systemService->registerSysPin(serviceName.c_str(), "pin");
    ServiceCommands *cmd = commandMgr->getServiceCommands(serviceName.c_str());
    logger = logMgr->newLogger(serviceName.c_str());

    initCommands(cmd);

    // defaults
    pin.setPin(-1);
    isEnabled = true;
    isReversed = false;
    isPullDown = false;
    isPullUp = false;
    isTouch = false;
    glitchMillis = 10;
    dClickDelay = 300;
    tClickDelay = 300;
    longClickDelay = 2000;
    touchReadMeasure = 0x1000;
    touchValueThreshold = 75;
    touchMillisThreshold = 100;

    dClickHandlerCount = 0;
    tClickHandlerCount = 0;
    longClickHandlerCount = 0;

    // load config
    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    if (isTouch) {
        touchIsOn = false;
        touchAvg = 10 * 256;
        touchLongAvg = touchAvg;
        touchIsLowCount = 0;
        touchLastAdjustTm = millis();
        touchTimer.init(eventLoop, [this](UEventLoopTimer *timer) {
            if (pin == -1) {
                if (millis() - touchLastAdjustTm > 3600 * 24) {
                    touchLastAdjustTm = millis(); // just to keep it from lagging too much from current time when no pin is defined
                }
                return;
            }
            uint16_t val = touchRead(pin);
            if (val == 0 || val > 0x7fff) {
                // just ignore a 0 reading, or too big
                return;
            }
            // we'll work in 1/265th of a unit value returned by touchRead(), in order
            // to have a better precision when performing averages and divisions, while
            // not using floating point calculations.
            uint16_t touchValue = val * 256;
            bool isOn = (touchValue < touchAvg * (touchValueThreshold * 256 / 100) / 256);
            if (isOn) {
                ++touchIsLowCount;
            } else {
                touchAvg = (touchAvg * 7 + val * 256 * 1) / 8; // exponential averaging, factor = 1/8
                touchLongAvg = (touchLongAvg * 511 + val * 256 * 1) / 512; // exponential averaging, factor = 1/512
                touchIsLowCount = 0;
            }
            if ((touchIsOn && !isOn) || (!touchIsOn && isOn && touchIsLowCount * 30 > touchMillisThreshold)) {
                logger->trace("Touch {}, touchAvg: {}, touchValue: {}",
                    isOn ? "ON" : "OFF", touchAvg / 256, touchValue / 256);
                touchIsOn = isOn;
                Dfa::Input inp = isOn ? B_TURNED_ON : B_TURNED_OFF;
                logger->debug("Input from touch: {}", dfa.inputName(inp));
                dfa.handleInput(inp);
            }

            // Long-term auto-setting of touch measure time

            // Aim at a touchLongAvg between 40 and 80, ideally 60
            unsigned long tm = millis();
            if (tm - touchLastAdjustTm > 10000) { // check every 10 seconds
                if (touchLongAvg / 256 > 0 && (touchLongAvg / 256 > 80 || touchLongAvg / 256 < 40)) {
                    int m = (int32_t)touchReadMeasure * 60 / ((int32_t)touchLongAvg / 256);
                    // limit the touch read measure between 0x4000 and 0x200 (default is 0x1000)
                    if (m > 0x4000) {
                        m = 0x4000;
                    }
                    if (m < 0x200) {
                        m = 0x200;
                    }
                    logger->debug("Adjusting touch read measure from {} to {}, touchLongAvg was {}",
                        touchReadMeasure, m, touchLongAvg / 256);
                    touchAvg = touchAvg * m / touchReadMeasure;
                    touchLongAvg = touchLongAvg * m / touchReadMeasure;
                    touchReadMeasure = m;
                    touchSetCycles(touchReadMeasure, 0x1000);
                }
                touchLastAdjustTm = tm;
            }

        });
    } else {
    //    isrCtx = (IsrCtx *)heap_caps_malloc(sizeof(IsrCtx), MALLOC_CAP_INTERNAL);
        isrCtx = (IsrCtx *)malloc(sizeof(IsrCtx));
        isrCtx->id = id;
        isrCtx->pin = pin;
        isrCtx->state = LOW;
        isrCtx->irqCount = 0;
        isrCtx->lostOff = 0;
        isrCtx->lostOn = 0;
        isrCtx->isReversed = isReversed;
        eventLoop->initIsrData(&isrCtx->isrData);
        isrCtx->buttonEventType = eventLoop->getEventType("button-internal", serviceName.c_str());

        eventLoop->onEvent(isrCtx->buttonEventType, [this](UEvent *event) {
            Dfa::Input inp = event->dataInt == 1 ? B_TURNED_ON : B_TURNED_OFF;
            logger->trace("Input from ISR: {}", dfa.inputName(inp));
            dfa.handleInput(inp);
            return true;
        });
    }

    initialState = ButtonService::ButtonState::BUTTON_STATE_OFF;
    state = initialState;

    initDfa();

    if (pin != -1 && isEnabled) {
        isEnabled = false; // so that enable() enables
        enable();
    }
}

void ButtonService::SingleButtonService::enable()
{
    if (isEnabled || pin == -1) {
        return;
    }
    if (isTouch) {
        /*
        * Set cycles that measurement operation takes
        * The result from touchRead, threshold and detection
        * accuracy depend on these values. Defaults are
        * 0x1000 for measure and 0x1000 for sleep.
        * With default values touchRead takes 0.5ms
        * */
        touchSetCycles(touchReadMeasure, 0x1000);
        touchTimer.setInterval(30);
        touchLastAdjustTm = millis();
        initialState = BUTTON_STATE_OFF;
        logger->debug("Button {} (touch) enabled, initial state: {}", id, initialState);
    } else {
        auto mode = (isPullUp ? INPUT_PULLUP : isPullDown ? INPUT_PULLDOWN : INPUT);
        pinMode(pin, mode);
        delayMicroseconds(5);
        isrCtx->state = digitalRead(pin);
        initialState = (isrCtx->state == (isReversed ? LOW : HIGH) ? BUTTON_STATE_ON : BUTTON_STATE_OFF);
        attachInterruptArg(digitalPinToInterrupt(pin), buttonIsr, isrCtx, CHANGE);
        logger->debug("Button {} enabled, initial state: {}", id, initialState);
    }
    state = initialState;

    dfa.enable(initialState == BUTTON_STATE_OFF ? B_OFF : B_ON);
    isEnabled = true;
}

void ButtonService::SingleButtonService::disable()
{
    if (!isEnabled) {
        return;
    }
    if (isTouch) {
        touchTimer.cancelInterval();
        logger->debug("Button {} (touch) disabled", id);
    } else {
        detachInterrupt(pin);
        logger->debug("Button {} disabled", id);
    }
    dfa.disable();
    isEnabled = false;
}

void ButtonService::SingleButtonService::buttonIsr(void *arg)
{
    IsrCtx *ctx = (IsrCtx *)arg;
    ++ctx->irqCount;
    int state = digitalRead(ctx->pin);
    if (state != ctx->state) {
        ctx->state = state;
        // queue event from isr
        UEvent event;
        event.eventType = ctx->buttonEventType;
        event.dataInt = (state == (ctx->isReversed ? LOW : HIGH));
        bool sent = ctx->isrData.queueEventFromIsr(event);
        if (!sent) {
            if (event.dataInt == 0) {
                ++ctx->lostOff;
            } else {
                ++ctx->lostOn;
            }
        }
    }
}

int ButtonService::SingleButtonService::onEvent(ButtonEvent event,
            std::function<bool(int id, ButtonEvent event, bool alreadyHandled)> fn)
{
    int handle = callbacks.add(fn, event);
    switch (event) {
        case BUTTON_LONG_CLICK: ++longClickHandlerCount; break;
        case BUTTON_DCLICK: ++dClickHandlerCount; break;
        case BUTTON_TCLICK: ++tClickHandlerCount; break;
        default: break;
    }
    return handle;
}

void ButtonService::SingleButtonService::removeOnEvent(int handle)
{
    int idx = callbacks.find(handle);
    if (idx != -1) {
        ButtonEvent event = callbacks.getOther(idx);
        switch (event) {
            case BUTTON_LONG_CLICK: --longClickHandlerCount; break;
            case BUTTON_DCLICK: --dClickHandlerCount; break;
            case BUTTON_TCLICK: --tClickHandlerCount; break;
            default: break;
        }
        callbacks.remove(handle);
        return;
    }
}

void ButtonService::SingleButtonService::callCallbacks(ButtonEvent event)
{
    const char *s;
    switch (event) {
        case BUTTON_ON: s = "BUTTON_ON"; break;
        case BUTTON_OFF: s = "BUTTON_OFF"; break;
        case BUTTON_CLICK: s = "BUTTON_CLICK"; break;
        case BUTTON_LONG_CLICK: s = "BUTTON_LONG_CLICK"; break;
        case BUTTON_DCLICK: s = "BUTTON_DCLICK"; break;
        case BUTTON_TCLICK: s = "BUTTON_TCLICK"; break;
        default: s = "BUTTON_<unknown>";
    }
    logger->debug("Button{}: {}", id, s);
    bool isHandled = false;
    for (int i = 0; i < callbacks.size(); i++) {
        ButtonEvent callbackEvent = callbacks.getOther(i);
        if (event == callbackEvent) {
            bool h = callbacks.get(i)(id, event, isHandled);
            isHandled = isHandled || h;
        }
    }
}

void ButtonService::SingleButtonService::initDfa()
{
    dfa.init(eventLoop, logger);

    dfa.onInput([this](Dfa *dfa, Dfa::State state, Dfa::Input input) {

        if (state.is(B_OFF)) {

            if (input.is(B_TURNED_ON)) { // turned on
                onTs = millis();
                isClicked = false;
                callCallbacks(BUTTON_ON);
                if (longClickHandlerCount == 0 && dClickHandlerCount == 0 && tClickHandlerCount == 0) {
                    // we don't need to wait for button off to consider a click
                    callCallbacks(BUTTON_CLICK);
                    isClicked = true;
                }
                return dfa->transitionTo(B_ON_DEBOUNCE);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(B_ON_DEBOUNCE)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                dfa->setStateTimeout(glitchMillis);
                return dfa->noTransition();
            } else if (input.is(B_TURNED_OFF)) {
                return dfa->transitionTo(B_OFF_DEBOUNCE);
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                return dfa->transitionTo(B_ON);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(B_OFF_DEBOUNCE)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                dfa->setStateTimeout(glitchMillis);
                return dfa->noTransition();
            } else if (input.is(B_TURNED_ON)) {
                return dfa->transitionTo(B_ON_DEBOUNCE);
            } else if (input.is(Dfa::Input::TIMEOUT)) { // turned off
                offTs = millis();
                callCallbacks(BUTTON_OFF);
                if (!isClicked) {
                    callCallbacks(BUTTON_CLICK);
                    isClicked = true;
                }
                return dfa->transitionTo(B_OFF);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(B_ON)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                if (longClickHandlerCount > 0) {
                    dfa->setStateTimeout(longClickDelay);
                }
                return dfa->noTransition();
            } else if (input.is(B_TURNED_OFF)) { // turned off
                offTs = millis();
                callCallbacks(BUTTON_OFF);
                if (!isClicked) {
                    callCallbacks(BUTTON_CLICK);
                    isClicked = true;
                }
                return dfa->transitionTo(B_OFF_2_DEBOUNCE);
            } else if (input.is(Dfa::Input::TIMEOUT)) { // longClickDelay passed
                callCallbacks(BUTTON_LONG_CLICK);
                isClicked = true; // no longer BUTTON_CLICK if we had a BUTTON_LONG_CLICK
                return dfa->noTransition(); // we'll wait button off, no new timeout
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(B_OFF_2_DEBOUNCE)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                dfa->setStateTimeout(glitchMillis);
                return dfa->noTransition();
            } else if (input.is(B_TURNED_ON)) {
                return dfa->transitionTo(B_ON_2_DEBOUNCE);
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                return dfa->transitionTo(B_OFF);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(B_ON_2_DEBOUNCE)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                dfa->setStateTimeout(glitchMillis);
                return dfa->noTransition();
            } else if (input.is(B_TURNED_OFF)) {
                return dfa->transitionTo(B_OFF_2_DEBOUNCE);
            } else if (input.is(Dfa::Input::TIMEOUT)) { // turned on
                onTs = millis();
                isClicked = false;
                callCallbacks(BUTTON_ON);
                if (longClickHandlerCount == 0 && dClickHandlerCount == 0 && tClickHandlerCount == 0) {
                    callCallbacks(BUTTON_CLICK);
                    isClicked = true;
                }
                return dfa->transitionTo(B_ON);
            } else {
                return dfa->transitionError();
            }

        } else {
            return dfa->transitionError();
        }

    });

}

void ButtonService::SingleButtonService::initCommands(ServiceCommands *cmd)
{
    cmd->registerSysPinData(
        ServiceCommands::SysPinDataBuilder(&pin)
        .help("--> Button pin -- change requires reboot")
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("isTouch", true)
        .cmd("isTouch")
        .help("--> Set to 'on' to use as touch button -- change requires reboot")
        .ptr(&isTouch)
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("reversed", true)
        .cmd("reversed")
        .help("--> Set to true if the logic is reversed, OFF state is high and ON state is low -- change requires reboot")
        .ptr(&isReversed)
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("pull", true)
        .cmd("pull")
        .help("--> up|down|none -- change requires reboot")
        .setFn([this](const String &val, bool isLoading, String *msg) {
            if (val.equalsIgnoreCase("UP")) {
                isPullUp = true;
                isPullDown = false;
            } else if (val.equalsIgnoreCase("DOWN")) {
                isPullUp = false;
                isPullDown = true;
            } else if (val.equalsIgnoreCase("NONE")) {
                isPullUp = false;
                isPullDown = false;
            } else {
                *msg = "Expecting UP, DOWN or NONE";
            }
            return true;
        })
        .getFn([this](String *val) {
            if (isPullUp) {
                *val = "UP";
            } else if (isPullDown) {
                *val = "DOWN";
            } else {
                *val = "NONE";
            }
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("touchReadMeasure", true)
        .cmd("touchReadMeasure")
        .help("--> Measure cycles, default is 0x1000 (4096). Adjusted automatically.")
        .isPersistent(false)
        .getFn([this]() {
            return this->touchReadMeasure;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("touchValueThreshold", true)
        .cmd("touchValueThreshold")
        .help("--> Threshold of considering a press, 10 - 90, defaults to 75")
        .ptr(&touchValueThreshold)
        .vMin(10)
        .vMax(90)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("touchMillisThreshold", true)
        .cmd("touchMillisThreshold")
        .help("--> Time in millis the touch must be below touchValueThreshold to be considered as ON, default 100")
        .ptr(&touchMillisThreshold)
        .vMin(10)
        .vMax(10000)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("debounceMillis", true)
        .cmd("debounceMillis")
        .help("--> Debounce time in millis")
        .vMin(1)
        .vMax(10000)
        .ptr(&glitchMillis)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("dClickMillis", true)
        .cmd("dClickMillis")
        .help("--> Double click delay in millis")
        .vMin(1)
        .vMax(10000)
        .ptr(&dClickDelay)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("tClickMillis", true)
        .cmd("tClickMillis")
        .help("--> Triple click delay in millis")
        .vMin(1)
        .vMax(10000)
        .ptr(&tClickDelay)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("longClickMillis", true)
        .cmd("longClickMillis")
        .help("--> Long click delay in millis")
        .vMin(1)
        .vMax(10000)
        .ptr(&longClickDelay)
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("enable", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .helpOn("--> Enable the button")
        .helpOff("--> Disable the button")
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (isLoading) {
                isEnabled = val;
            } else {
                if (val == isEnabled) {
                    if (isEnabled) {
                        *msg = "Already enabled";
                    } else {
                        *msg = "Already disabled";
                    }
                } else {
                    if (val) {
                        enable();
                        *msg = "Enabled";
                    } else {
                        disable();
                        *msg = "Disabled";
                    }
                }
            }
            return true;
        })
        .getFn([this]() {
            return isEnabled;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("irqCount", true)
        .cmd("irqCount")
        .help("--> IRQ count")
        .isPersistent(false)
        .getFn([this]() {
            return isTouch ? 0 : isrCtx->irqCount;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("buttonState", true)
        .cmd("buttonState")
        .help("--> Button state")
        .isPersistent(false)
        .getFn([this](String *val) {
            *val = isTouch ? "N/A" : (isrCtx->state == (isReversed ? LOW : HIGH) ? "ON" : "OFF");
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("touchAvg", true)
        .cmd("touchAvg")
        .help("--> Average touch reading")
        .isPersistent(false)
        .getFn([this](String *val) {
            *val = (float)(isTouch ? ((float)touchAvg / 256) : -1.0f);
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("touchLongAvg", true)
        .cmd("touchLongAvg")
        .help("--> Average touch reading with a longer time constant")
        .isPersistent(false)
        .getFn([this](String *val) {
            *val = (float)(isTouch ? ((float)touchLongAvg / 256) : -1.0f);
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("dfaState", true)
        .cmd("dfaState")
        .help("--> DF state")
        .isPersistent(false)
        .getFn([this](String *val) {
            *val = dfa.stateName(dfa.getState());
        })
    );

    cmd->onAfterStatus([this](String *msg) {
        if (!isTouch) {
            *msg += "    lostOff: "; *msg += isrCtx->lostOff;
            *msg += "\n    lostOn: "; *msg += isrCtx->lostOn;
        }
    });

}

ButtonService::ButtonState ButtonService::SingleButtonService::getInitialState()
{
    return initialState;
}

ButtonService::ButtonState ButtonService::SingleButtonService::getState()
{
    return state;
}


/************************
 *
 * Button Service
 *
 ************************/

void ButtonService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr, SystemService *systemService)
{
    ServiceCommands *cmd = commandMgr->getServiceCommands("button");
    initCommands(cmd);

    // defaults
    buttonCount = 1;

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
    for (int i = 0; i < buttonCount; i++) {
        SingleButtonService *b = new SingleButtonService(i);
        buttons.push_back(b);
        b->init(eventLoop, commandMgr, logMgr, systemService);
    }

}

void ButtonService::initCommands(ServiceCommands *cmd)
{
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("count", true)
        .cmd("count")
        .help("--> Number of buttons -- change requires reboot")
        .vMin(0)
        .vMax(100)
        .ptr(&buttonCount)
    );

}

ButtonService::SingleButtonService *ButtonService::getButton(int id)
{
    if (id < 0 || id >= buttonCount) {
        return nullptr;
    }
    return buttons[id];
}

#endif
