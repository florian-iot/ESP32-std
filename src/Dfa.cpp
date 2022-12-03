#include <CompilationOpts.h>

#include <functional>
#include "Dfa.h"
#include "UEvent.h"
#include "LogMgr.h"


Dfa::Input Dfa::Input::NONE(0xFFFFFFFF);
Dfa::Input Dfa::Input::TIMEOUT(0xFFFFFFFE);
Dfa::Input Dfa::Input::ENTER_STATE(0xFFFFFFFD);
Dfa::State Dfa::State::NO_STATE(0xFFFFFFFF);
Dfa::State Dfa::State::TRANSITION_ERROR(0xFFFFFFFE);

Dfa::Dfa(const char *c_dfaName, int c_dfaId)
: dfaName(c_dfaName), dfaId(c_dfaId << 16), state(Dfa::State::NO_STATE), scheduledInput(Dfa::Input::NONE)
{
}

Dfa::Dfa()
: dfaName("dfa"), dfaId(1 << 16), state(Dfa::State::NO_STATE), scheduledInput(Dfa::Input::NONE)
{
}

void Dfa::init(UEventLoop *eventLoop, Logger *logger)
{
    this->eventLoop = eventLoop;
    this->logger = logger;
    isEnabled = false;
}

void Dfa::init(UEventLoop *eventLoop, Logger *logger, State initialState)
{
    init(eventLoop, logger);
    enable(initialState);
}

void Dfa::enable(State initialState)
{
    if (isEnabled) {
        return;
    }
    eventLoop->registerTimer(&timer);
    timer.setCallback([this](UEventLoopTimer *timer) {
        handleInput(Input::TIMEOUT);
    });
    eventLoop->registerTimer(&inputScheduleTimer);
    inputScheduleTimer.setCallback([this](UEventLoopTimer *timer) {
        handleInput(scheduledInput);
    });
    state = initialState;
    isHandlingInput = false;
    transitionTs = esp_timer_get_time();
    // send the first Dfa::Input::ENTER_STATE, for the first state
    scheduledInput = Dfa::Input::ENTER_STATE;
    inputScheduleTimer.setTimeoutMicros(1);
    if (!isOwnState(initialState)) {
        if (logger) {
            logger->error("Dfa {}({}) initialState from another Dfa ({})!",
                dfaName, dfaId >> 16, initialState.getDfaId());
        }
        initialState = State::NO_STATE;
    }
    if (logger) {
        logger->debug("DFA {} initialized at state {}\n", dfaName,
            this->stateName(state));
    }
    isEnabled = true;
}

void Dfa::disable()
{
    if (!isEnabled) {
        return;
    }
    timer.cancelTimeout();
    timer.unregister();
    inputScheduleTimer.cancelTimeout();
    inputScheduleTimer.unregister();
    isEnabled = false;
}

void Dfa::terminate()
{
    disable();
}

const char *Dfa::name() {
    return dfaName;
}

const char *Dfa::inputName(Input inp)
{
    if (!isOwnInput(inp)) {
        if (logger) {
            logger->error("In Dfa {}({}) encountered input from another Dfa: {}",
                dfaName, dfaId >> 16, inp.getDfaId());
        }
        return "From other Dfa!!!";
    }
    int v = inp.getId();
    if (v >= 0 && v < inputNames.size()) {
        return inputNames[v];
    }
    if (inp == Dfa::Input::TIMEOUT) {
        return "TIMEOUT";
    } else if (inp == Dfa::Input::ENTER_STATE) {
        return "ENTER_STATE";
    } else {
        return "<unknown>";
    }
}

const char *Dfa::stateName(State state)
{
    if (!isOwnState(state)) {
        if (logger) {
            logger->error("In Dfa {}({}) encountered stat from another Dfa: {}",
                dfaName, dfaId >> 16, state.getDfaId());
        }
        return "From another Dfa!!!";
    }
    int v = state.getId();
    if (state == Dfa::State::NO_STATE) {
        return "NO_STATE";
    } else if (state == Dfa::State::TRANSITION_ERROR) {
        return "TRANSITION_ERROR";
    } else if (v >= 0 && v < stateNames.size()) {
        return stateNames[v];
    } else {
        return "<unknown>";
    }
}

void Dfa::setStateTimeout(int timeoutMillis)
{
    timer.setTimeout(timeoutMillis);
    if (logger) {
        logger->debug("Dfa {} setting timeout to {}", dfaName, timeoutMillis);
    }
}

int Dfa::getStateTimeout()
{
    return timer.getTimeout();
}


void Dfa::clearStateTimeout()
{
    timer.cancelTimeout();
    if (logger) {
        logger->debug("Dfa {} clearing timeout", dfaName);
    }
}

void Dfa::onInput(std::function<Dfa::TransitionInfo (Dfa *dfa, State state, Input input)> callback)
{
    callbackList.push_back(callback);
}

bool Dfa::handleInput(Input inp)
{
    if (!isOwnInput(inp)) {
        if (logger) {
            logger->error("Dfa {} ::handleInput() called with an input from another Dfa ({})!",
                dfaName, inp.getDfaId());
        }
    }
    if (isHandlingInput) {
        if (logger) {
            logger->error("Dfa {} ::handleInput() called from within handling an input, this is not allowed.",
                dfaName);
        }
        return false;
    }
    isHandlingInput = true;

    if (logger && !(inp == Dfa::Input::ENTER_STATE)) {
        logger->debug("Dfa {} handling input {}-{}, state: {}",
            dfaName, inp.getId(),
            inputName(inp),
            stateName(state));
    }

    Input currentInput = inp;
    bool processed;
    State oldState = state;
    bool doLoop;
    do {
        doLoop = false;
        processed = false;

        TransitionInfo ti(State::NO_STATE, -1);
        for (int i = 0; !processed && i < callbackList.size(); i++) {
            ti = callbackList[i](this, state, currentInput);
            processed = (ti.newState != State::NO_STATE);
        }
        if (logger) {
            if (!processed) {
                if (!(currentInput == Dfa::Input::ENTER_STATE)) {
                    logger->debug("Dfa {} handled input {}-{}: no state change, state: {}",
                        dfaName, currentInput.getId(),
                        inputName(currentInput),
                        stateName(state));
                }
            } else if (ti.newState == State::TRANSITION_ERROR) {
                if (currentInput != Input::ENTER_STATE) { // for a TRANSITION_ERROR on ENTER_STATE, log nothing, it is simply a non-handled ENTER_STATE
                    logger->debug("Dfa {} handled input {}-{}: transition error, no state change, state: {}",
                        dfaName, currentInput.getId(),
                        inputName(currentInput),
                        stateName(state));
                }
            } else {
                logger->debug("Dfa {} handled input {}-{}: old state: {}, new state: {}, timeout {}",
                    dfaName, currentInput.getId(),
                    inputName(currentInput),
                    stateName(state),
                    stateName(ti.newState),
                    ti.newTimeout);
            }
        }

        if (ti.newState != State::NO_STATE && ti.newState != State::TRANSITION_ERROR) { // transition to new state
            state = ti.newState;
            inputScheduleTimer.cancelTimeout();
            if (ti.newTimeout >= 0) {
                timer.setTimeout(ti.newTimeout);
            } else {
                timer.cancelTimeout();
            }

            // handle input ENTER_STATE, if we've transitioned to new state
            if (!(state == oldState)) { // we're transitioning to a new state, process an ENTER_STATE input
                transitionTs = esp_timer_get_time();
                currentInput = Input::ENTER_STATE;
                oldState = state;
                doLoop = true;
            }
        }

        if (!doLoop) { // if we finished on processing one input, see whether at the final state we've queued inputs
            Input newInput = getNextInputForState(state);
            if (newInput != Input::NONE) {
                logger->debug("Dfa {} handling queued input {}-{}, state: {}",
                    dfaName, newInput.getId(),
                    inputName(newInput),
                    stateName(state));
                transitionTs = esp_timer_get_time();
                currentInput = newInput;
                oldState = state;
                doLoop = true;
            }
        }

    } while (doLoop);

    isHandlingInput = false;
    return processed;
}

void Dfa::queueInput(Input input, int millis)
{
    if (!isOwnInput(input)) {
        if (logger) {
            logger->error("Dfa {} initialState from another Dfa ({})!", dfaName, input.getDfaId());
        }
        input = Input::NONE;
    }
    timer.cancelTimeout();
    scheduledInput = input;
    inputScheduleTimer.setTimeout(millis);
    if (logger) {
        logger->debug("Dfa {} setting schedule input {}-{} with timeout to {}",
            dfaName, input.getId(),
            inputName(input), millis);
    }
}

Dfa::Input Dfa::peekNextInputForState(State st)
{
    for (auto i = inputForState.begin(); i != inputForState.end(); i++) {
        if (i->state == st) {
            return i->input;
        }
    }
    return Input::NONE;
}

Dfa::Input Dfa::getNextInputForState(State st)
{
    for (auto i = inputForState.begin(); i != inputForState.end(); i++) {
        if (i->state == st) {
            inputForState.erase(i);
            return i->input;
        }
    }
    return Input::NONE;
}

void Dfa::queueInputForState(Input inp, State st)
{
    if (isHandlingInput || state != st || peekNextInputForState(st) != Input::NONE) {
        inputForState.push_back(InputForState(inp, st));
    } else {
        (void) handleInput(inp);
    }
}

int64_t Dfa::getMillisInState()
{
    return (int32_t)(esp_timer_get_time() - transitionTs) / 1000;
}

