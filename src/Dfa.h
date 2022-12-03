#include <CompilationOpts.h>
#ifndef INCL_DFA_H
#define INCL_DFA_H

#include <functional>
#include "UEvent.h"
#include "LogMgr.h"
#include "deque"

class Dfa {
public:
    class Input {
    friend class Dfa;
        int32_t input;
        Input(int v);
    public:
        static Input NONE;
        static Input TIMEOUT;
        static Input ENTER_STATE;

        Input(const Input &other);
        bool operator==(Input other);
        bool operator!=(Input other);
        int getDfaId();
        int getId();
        bool is(Input v1);
        bool is(Input v1, Input v2);
        bool is(Input v1, Input v2, Input v3);
        bool is(Input v1, Input v2, Input v3, Input v4);
        bool is(Input v1, Input v2, Input v3, Input v4, Input v5);
        bool is(Input v1, Input v2, Input v3, Input v4, Input v5, Input v6);
    };
    class State {
    friend class Dfa;
        int32_t state;
        State(int v);
    public:
        static State NO_STATE;
        static State TRANSITION_ERROR;

        State(const State &other);
        bool operator==(State other);
        bool operator!=(State other);
        int getDfaId();
        int getId();
        bool is(State v1);
        bool is(State v1, State v2);
        bool is(State v1, State v2, State v3);
        bool is(State v1, State v2, State v3, State v4);
        bool is(State v1, State v2, State v3, State v4, State v5);
        bool is(State v1, State v2, State v3, State v4, State v5, State v6);
    };
    class TransitionInfo {
    friend class Dfa;
    private:
        State newState; // NO_STATE if not processed
        int newTimeout; // -1 if no timeout

        TransitionInfo(State newState) : newState(newState) { this->newTimeout = -1; }
        TransitionInfo(State newState, int newTimeout) : newState(newState) { this->newTimeout = newTimeout; }
        TransitionInfo &operator=(const TransitionInfo &i) { this->newState = i.newState; this->newTimeout = i.newTimeout; return *this; }
    public:
        TransitionInfo(const TransitionInfo &i) : newState(i.newState) { this->newTimeout = i.newTimeout; }
    };
private:
    UEventLoop *eventLoop;
    /** logger may be null, in which case no logging should be done */
    Logger *logger;
    const char *dfaName;
    int dfaId;
    std::vector<const char *> stateNames;
    std::vector<const char *> inputNames;

    bool isEnabled;
    State state;
    std::vector<std::function<TransitionInfo (Dfa *dfa, State state, Input input)> > callbackList;
    bool isHandlingInput;
    UEventLoopTimer timer;
    UEventLoopTimer inputScheduleTimer;
    Input scheduledInput;
    int64_t transitionTs; // transition into current state from another state

    struct InputForState {
        State state;
        Input input;
        InputForState(Input inp, State st) : state(st), input(inp) { }
        InputForState() : state(Dfa::State::NO_STATE), input(Dfa::Input::NONE) { }
    };
    std::deque<InputForState> inputForState;
    Input peekNextInputForState(State st);
    Input getNextInputForState(State st);

public:
    // dfaId must be initialized at construction, so that nextInput() and nextState() work
    // correctly when used as default initializers in a class that uses Dfa.
    Dfa(const char *dfaName, int dfaId);
    // use when there's no need to distinguish between Dfa-s in a class
    Dfa();
    // initialize but do not enable yet; logger can be null in order not to generate any log
    void init(UEventLoop *eventLoop, Logger *logger);
    // initialize and enable; logger can be null in order not to generate any log
    void init(UEventLoop *eventLoop, Logger *logger, State initialState);
    void enable(State initialState);
    void disable();
    void terminate();

//    void stateHandler(Dfa::State state, std::function<TransitionInfo (Dfa::Input input)> handler);

    Dfa::State nextState(const char *name);
    Dfa::Input nextInput(const char *name);
    const char *name();
    const char *inputName(Input v);
    const char *stateName(State v);
    bool isOwnState(State state);
    bool isOwnInput(Input input);

    // The callback must return the result of DFA::transitionTo() if the input is processed, DFA::transitionError() otherwise.
    // Many callbacks can be registered and tried in turn as long as the input is not processed.
    void onInput(std::function<TransitionInfo (Dfa *dfa, State state, Input input)> callback);

    // Handle an input event
    // Returns true if the input is handled. An error log is made if the input is not handled.
    bool handleInput(Input input);

    // Sets a timeout which, after expiring, generates an input event
    void queueInput(Input input, int millis);

    // When the state is or becomes the specified one, the given input is sent.
    // The input is sent after ENTER_STATE (and only if the state remains the same).
    void queueInputForState(Input input, State state);

    State getState();
    int64_t getMillisInState();
    int getStateTimeout();

    // Transitions to new state, and set a timeout if timeoutMillis is set; clears any scheduled input
    TransitionInfo transitionTo(State state, int timeoutMillis = -1);
    TransitionInfo noTransition();
    TransitionInfo transitionError();
    void setStateTimeout(int timeoutMillis);
    void clearStateTimeout();
};

inline Dfa::Input::Input(int32_t v)
{
    input = v;
}

inline Dfa::Input::Input(const Input &other)
{
    input = other.input;
}

inline bool Dfa::Input::operator==(Input other)
{
    return input == other.input;
}

inline bool Dfa::Input::operator!=(Input other)
{
    return input != other.input;
}

inline int Dfa::Input::getDfaId() { return ((uint32_t)input) >> 16; }
inline int Dfa::Input::getId() { return input & 0xFFFF; }

inline bool Dfa::Input::is(Input v1) { return input == v1.input; }
inline bool Dfa::Input::is(Input v1, Input v2) { return input == v1.input || input == v2.input; }
inline bool Dfa::Input::is(Input v1, Input v2, Input v3) { return input == v1.input || input == v2.input || input == v3.input; }
inline bool Dfa::Input::is(Input v1, Input v2, Input v3, Input v4) { return input == v1.input || input == v2.input || input == v3.input || input == v4.input; }
inline bool Dfa::Input::is(Input v1, Input v2, Input v3, Input v4, Input v5) { return input == v1.input || input == v2.input || input == v3.input || input == v4.input || input == v5.input; }
inline bool Dfa::Input::is(Input v1, Input v2, Input v3, Input v4, Input v5, Input v6) { return input == v1.input || input == v2.input || input == v3.input || input == v4.input || input == v5.input || input == v6.input; }


inline Dfa::State::State(int32_t v)
{
    state = v;
}

inline Dfa::State::State(const State &other)
{
    state = other.state;
}

inline bool Dfa::State::operator==(State other)
{
    return state == other.state;
}

inline bool Dfa::State::operator!=(State other)
{
    return state != other.state;
}

inline int Dfa::State::getDfaId() { return ((uint32_t)state) >> 16; }
inline int Dfa::State::getId() { return (int)(int16_t)(state & 0xFFFF); }

inline bool Dfa::State::is(State v1) { return state == v1.state; }
inline bool Dfa::State::is(State v1, State v2) { return state == v1.state || state == v2.state; }
inline bool Dfa::State::is(State v1, State v2, State v3) { return state == v1.state || state == v2.state || state == v3.state; }
inline bool Dfa::State::is(State v1, State v2, State v3, State v4) { return state == v1.state || state == v2.state || state == v3.state || state == v4.state; }
inline bool Dfa::State::is(State v1, State v2, State v3, State v4, State v5) { return state == v1.state || state == v2.state || state == v3.state || state == v4.state || state == v5.state; }
inline bool Dfa::State::is(State v1, State v2, State v3, State v4, State v5, State v6) { return state == v1.state || state == v2.state || state == v3.state || state == v4.state || state == v5.state || state == v6.state; }

inline Dfa::Input Dfa::nextInput(const char *name)
{
    int v = inputNames.size();
    inputNames.push_back(name);
    return Input(v | dfaId);
}

inline Dfa::State Dfa::nextState(const char *name)
{
    int v = stateNames.size();
    stateNames.push_back(name);
    return State(v | dfaId);
}

inline bool Dfa::isOwnState(State state)
{
    return (state.state & 0xFFFF0000) == dfaId || state.state < 0;
}

inline bool Dfa::isOwnInput(Input input)
{
    return (input.input & 0xFFFF0000) == dfaId || input.input < 0;
}

inline Dfa::State Dfa::getState()
{
    return state;
}

inline Dfa::TransitionInfo Dfa::transitionError()
{
    return TransitionInfo(State::TRANSITION_ERROR);
}

inline Dfa::TransitionInfo Dfa::transitionTo(State state, int timeoutMillis)
{
    return TransitionInfo(state, timeoutMillis);
}

inline Dfa::TransitionInfo Dfa::noTransition()
{
    return TransitionInfo(State::NO_STATE, -1);
}

#endif
