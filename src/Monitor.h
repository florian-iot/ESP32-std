#ifndef INC_MONITOR_H
#define INC_MONITOR_H

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>

class Monitor {
public:
    Monitor();
    ~Monitor();
    bool enter();
    bool enter(int xTicksToWait);
    void leave();
private:
    EventGroupHandle_t eventGroup;
    int usedConditionBits;
friend class Condition; // next methods are used by Condition
    int allocateConditionBit();
    void releaseConditionBit(int bitMask);
};

class MonitorScope {
    Monitor *mon;
public:
    explicit MonitorScope(Monitor *mon);
    ~MonitorScope();
};

class Condition {
private:
    Monitor *mon;
    int bitMask;
    int waiting; // how many are already waiting in this condition
    bool isNotifyAll; // true if a notifyAll() was called and waiters are being woken up
public:
    explicit Condition(Monitor *mon);
    ~Condition();
    bool wait();
    bool wait(int xTicksToWait);
    void notify();
    void notifyAll();
};

inline Monitor::Monitor()
{
    eventGroup = xEventGroupCreate();
    xEventGroupSetBits(eventGroup, 0b01);
    usedConditionBits = 0;
}

inline Monitor::~Monitor()
{
    if (eventGroup != nullptr) {
        vEventGroupDelete(eventGroup);
    }
}

inline bool Monitor::enter()
{
    return enter(portMAX_DELAY);
}

inline bool Monitor::enter(int xTicksToWait)
{
    EventBits_t rc = xEventGroupWaitBits(eventGroup, 0b01, pdTRUE, pdTRUE, xTicksToWait);
    return (rc & 0b01) == 0b01; // we did not time out if the bit 0 was set
}

inline void Monitor::leave()
{
    xEventGroupSetBits(eventGroup, 0b01);
}

inline int Monitor::allocateConditionBit()
{
    ++usedConditionBits;
    if (usedConditionBits >= 7) {
        abort(); // for now we only allocate bits statically, never reclaim them
        // conditions must be created at the same time as the monitor, only 7 conditions can be created
    }
    return 1 << (usedConditionBits + 1);
}

inline void Monitor::releaseConditionBit(int bitMask)
{
    // noop for the time being, dynamic creation of conditions not supported
}

inline MonitorScope::MonitorScope(Monitor *mon)
:   mon(mon)
{
    mon->enter();
}

inline MonitorScope::~MonitorScope()
{
    mon->leave();
}

// Condition

inline Condition::Condition(Monitor *mon)
:
    mon(mon),
    bitMask(mon->allocateConditionBit()),
    waiting(0),
    isNotifyAll(false)
{
    if (bitMask == 0) {
        // not enough bits, we didn't manage to initialize the condition...
        abort();
    }
}

inline Condition::~Condition()
{
    mon->releaseConditionBit(bitMask);
}

inline bool Condition::wait()
{
    return wait(portMAX_DELAY);
}

inline bool Condition::wait(int xTicksToWait)
{
    ++waiting;

    bool timedOut = false;
    EventBits_t rc = xEventGroupSync(mon->eventGroup, 0b01, 0b01 | bitMask, xTicksToWait);
    if ((rc & (0b01 | bitMask)) != (0b01 | bitMask)) {
        timedOut = true;
    }

    --waiting;

    if (!timedOut) {
        if (isNotifyAll && waiting != 0) { // if more are waiting on this condition, make sure they're all awaken
            // release monitor and re-signal the condition, this will wake up one waiter
            xEventGroupSetBits(mon->eventGroup, 0b01 | bitMask);
            // then get the monitor again
            EventBits_t rc = xEventGroupWaitBits(mon->eventGroup, 0b01, pdTRUE, pdTRUE, portMAX_DELAY);
            assert((rc & 0b01) == 0b01);
        } else { // else we've the monitor, no other waiters to wake up, we're done, just clean up notifyAll flag
            isNotifyAll = false;
            // condition bit has been cleared
        }
    } else {
        // if timed out, we need to reenter the monitor
        // if we did not time out, the monitor was reentered (we waited for bit 0 then cleared it)
        EventBits_t rc = xEventGroupWaitBits(mon->eventGroup, 0b01, pdTRUE, pdTRUE, portMAX_DELAY);
        assert((rc & 0b01) == 0b01);
    }

    return timedOut;
}

inline void Condition::notify()
{
    if (waiting > 0) {
        xEventGroupSetBits(mon->eventGroup, bitMask);
    }
}

inline void Condition::notifyAll()
{
    if (waiting > 0) {
        isNotifyAll = true;
        xEventGroupSetBits(mon->eventGroup, bitMask);
    }
}


#endif
