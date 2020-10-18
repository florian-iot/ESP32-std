#ifndef INCL_UEVENT_H
#define INCL_UEVENT_H

#include "CompilationOpts.h"

#include <functional>
#include <WString.h>
#include <functional>
#include <FreeRTOS.h>
#include <freertos/queue.h>
#include <vector>
#include <unordered_map>
//#include <stdatomic.h>
#include <atomic>
#include <Monitor.h>
#include <TaskScheduler.h>

/*

Event loop on main thread:
- queueEvent()
- sleeps with timeout the sleep duration of the scheduler
- onEvent(callback)
- setTimeout(callback, timeout), setInterval(callback, interval [, firstInterval] )

From other threads:
- postEvent()

An event may have a onProcessed callback, called when event processing completed.

When posting an event from another thread, the posting
thread may wait to be notified when processing is done, along
with the processing result.

An event and eventually the processing result are passed around by value.


*/

class UEventLoop;

struct UEvent {
    uint32_t eventType;
    union {
        void *dataPtr;
        int64_t dataInt;
    };
    UEvent() { eventType = -1; }
    UEvent(uint32_t eventType) { this->eventType = eventType; }
    UEvent(uint32_t eventType, void *data) { this->eventType = eventType; this->dataPtr = data; }
    UEvent(uint32_t eventType, int data) { this->eventType = eventType; this->dataInt = data; }
    UEvent(const UEvent &other) { memcpy(this, &other, sizeof(UEvent)); }
};

class UEventHandle_t {
private:
    int handle;
    explicit UEventHandle_t(int h) { handle = h; }
friend class UEventLoop;
public:
    UEventHandle_t() { handle = -1; }
    UEventHandle_t(const UEventHandle_t &other) { handle = other.handle; }
};



struct UEventEntry {
    UEvent event;
    SemaphoreHandle_t sem;
    std::function<void(UEvent *event, bool isProcessed)> onProcess;
    std::function<void(UEvent*)> finalizer;
    bool isProcessed;
    UEventEntry(): event(), sem(nullptr), onProcess(nullptr), finalizer(nullptr), isProcessed(false) { }
};


//  █████  █████ ██████████                                  █████    █████
// ░░███  ░░███ ░░███░░░░░█                                 ░░███    ░░███
//  ░███   ░███  ░███  █ ░  █████ █████  ██████  ████████   ███████   ░███         ██████   ██████  ████████
//  ░███   ░███  ░██████   ░░███ ░░███  ███░░███░░███░░███ ░░░███░    ░███        ███░░███ ███░░███░░███░░███
//  ░███   ░███  ░███░░█    ░███  ░███ ░███████  ░███ ░███   ░███     ░███       ░███ ░███░███ ░███ ░███ ░███
//  ░███   ░███  ░███ ░   █ ░░███ ███  ░███░░░   ░███ ░███   ░███ ███ ░███      █░███ ░███░███ ░███ ░███ ░███
//  ░░████████   ██████████  ░░█████   ░░██████  ████ █████  ░░█████  ███████████░░██████ ░░██████  ░███████
//   ░░░░░░░░   ░░░░░░░░░░    ░░░░░     ░░░░░░  ░░░░ ░░░░░    ░░░░░  ░░░░░░░░░░░  ░░░░░░   ░░░░░░   ░███░░░
//                                                                                                  ░███
//                                                                                                  █████
//                                                                                                 ░░░░░

class UEventLoopTimer;

class UEventLoop {
    friend class UEventLoopTimer;
    friend class Esp32Timer;
private:
    const char *loopName;
    QueueHandle_t queue;
    volatile TaskHandle_t loopTask;
    Monitor mon;
    class ProcessorEntry {
        int id;
        uint32_t eventType;
        std::function<bool(UEvent *event)> processor;
        bool matchesType(uint32_t eventType);
    friend class UEventLoop;
    };
    int processorSequence;
    std::vector<ProcessorEntry> processors;
    Scheduler scheduler;
    std::atomic_bool isShuttingDown;
    struct EventTypeEntry {
        char *eventClass;
        char *eventName;
        uint32_t eventType;
        EventTypeEntry(char *eventClass, char *eventName, uint32_t eventType) {
            this->eventClass = eventClass;
            this->eventName = eventName;
            this->eventType = eventType;
        }
        EventTypeEntry(const EventTypeEntry &other) {
            this->eventClass = other.eventClass;
            this->eventName = other.eventName;
            this->eventType = other.eventType;
        }
    };
    uint16_t nextEventClassType;
    uint16_t nextEventNameType;
    std::vector<char *> eventClasses;
    std::vector<EventTypeEntry> eventNames;

    void handleEvent(UEventEntry *eventEntry);
    bool matchesType(int eventType, ProcessorEntry p);
    uint32_t getEventClassTypeInternal(const char *eventClass);
public:
    UEventLoop(const char *loopName);
    ~UEventLoop();

    /** Runs the event loop, quits only after shutdown() is called */
    void run();
    bool runOnce(long timeoutMillis);
    /**
     * Can be called asynchronously, or from within the event handlers of this loop.
     */
    void shutdown();

    xTaskHandle getProcessingTask();

    /**
     * Register the event class in case it wasn't yet registered, return its ID
     */
    uint32_t getEventClassType(const char *eventClass);
    /**
     * Register the event in case it wasn't yet registered, return its ID
     */
    uint32_t getEventType(const char *eventClass, const char *eventName);
    /**
     * Returns -1 if the event isn't registered
     */
    uint32_t findEventClassType(const char *eventClass);
    /**
     * Returns -1 if the event isn't registered
     */
    uint32_t findEventType(const char *eventClass, const char *eventName);
    /**
     * The event type must have beeen registered
     */
    const char *getEventClass(uint32_t eventType);
    /**
     * The event type must have beeen registered
     */
    const char *getEventName(uint32_t eventType);

    UEventHandle_t onEvent(uint32_t eventType, std::function<bool(UEvent *event)> const processor);
    void unregister(UEventHandle_t eventHandler);

    /** Process an event immediately */
    bool processEvent(UEvent event);
    /**
     * Queue an event, to be processed when its turn comes. If semaphore is
     * specified, a "give" will be performed on it after the event is consumed (processed or not).
     */
    bool queueEvent(UEvent &event, std::function<void(UEvent*)> finalizer, SemaphoreHandle_t = nullptr);
    bool queueEvent(UEvent &event, std::function<void(UEvent*)> finalizer,
            std::function<void(UEvent *event, bool isProcessed)> onProcess = nullptr, SemaphoreHandle_t = nullptr);

    void registerTimer(UEventLoopTimer *timer);
    void registerTimer(UEventLoopTimer *timer, std::function<void(*)(UEventLoop *)> callback);

    void unregisterTimer(UEventLoopTimer *timer);

};


//  █████  █████ ██████████                                  █████    █████                                   ███████████  ███
// ░░███  ░░███ ░░███░░░░░█                                 ░░███    ░░███                                   ░█░░░███░░░█ ░░░
//  ░███   ░███  ░███  █ ░  █████ █████  ██████  ████████   ███████   ░███         ██████   ██████  ████████ ░   ░███  ░  ████  █████████████    ██████  ████████
//  ░███   ░███  ░██████   ░░███ ░░███  ███░░███░░███░░███ ░░░███░    ░███        ███░░███ ███░░███░░███░░███    ░███    ░░███ ░░███░░███░░███  ███░░███░░███░░███
//  ░███   ░███  ░███░░█    ░███  ░███ ░███████  ░███ ░███   ░███     ░███       ░███ ░███░███ ░███ ░███ ░███    ░███     ░███  ░███ ░███ ░███ ░███████  ░███ ░░░
//  ░███   ░███  ░███ ░   █ ░░███ ███  ░███░░░   ░███ ░███   ░███ ███ ░███      █░███ ░███░███ ░███ ░███ ░███    ░███     ░███  ░███ ░███ ░███ ░███░░░   ░███
//  ░░████████   ██████████  ░░█████   ░░██████  ████ █████  ░░█████  ███████████░░██████ ░░██████  ░███████     █████    █████ █████░███ █████░░██████  █████
//   ░░░░░░░░   ░░░░░░░░░░    ░░░░░     ░░░░░░  ░░░░ ░░░░░    ░░░░░  ░░░░░░░░░░░  ░░░░░░   ░░░░░░   ░███░░░     ░░░░░    ░░░░░ ░░░░░ ░░░ ░░░░░  ░░░░░░  ░░░░░
//                                                                                                  ░███
//                                                                                                  █████
//                                                                                                 ░░░░░

class Esp32Timer {
    static void esp32TimerCallback(void *arg);

    esp_timer_handle_t handle;
    std::function<void (Esp32Timer *timer, uint64_t expectedTime)> callback;
    const char *name;

    uint64_t interval; // interval set with setInterval()
    uint64_t nextRun; // time when the next run is expected

    UEventLoop *eventLoop;
public:
    Esp32Timer(UEventLoop *eventLoop, const char *name);
    ~Esp32Timer();
    /** May not be called when the callback is active */
    void setCallback(std::function<void (Esp32Timer *timer, uint64_t expectedTime)> callback);
    void setTimeoutMicros(uint64_t timeout);
    void clearTimeout();
    void setIntervalMicros(uint64_t timeout);
    void clearInterval();

    static uint64_t currentTime();
};

class UEventLoopTimer {
private:
    UEventLoop *eventLoop;
    Task schedulerTask;
    std::function<void(UEventLoopTimer*)> callback;
    std::function<void()> scheduledCallback;

public:
    UEventLoopTimer(UEventLoop *eventLoop = nullptr, std::function<void(UEventLoopTimer *)> callback = nullptr);
    ~UEventLoopTimer();
    void init(UEventLoop *eventLoop, std::function<void(UEventLoopTimer *)> callback);
    void setCallback(std::function<void(UEventLoopTimer *)> callback);
    UEventLoop *getEventLoop();
    void setInterval(std::function<void(UEventLoopTimer *)> callback, long intervalMillis);
    void setInterval(long intervalMillis);
    void setIntervalMicros(std::function<void(UEventLoopTimer *)> callback, long intervalMicros);
    void setIntervalMicros(long intervalMicros);
    void cancelInterval();

    void setTimeout(std::function<void(UEventLoopTimer *)> callback, long intervalMillis);
    void setTimeout(long intervalMillis);
    void setTimeoutMicros(std::function<void(UEventLoopTimer *)> callback, long intervalMicros);
    void setTimeoutMicros(long intervalMicros);
    void cancelTimeout();

    bool isActive();
    /**
     * Returns the overrun of last execution (or current execution, if called within an execution). If
     * negative, that means that at the last execution, we were already late for the next execution.
     */
    long getOverrunMicros();
    void unregister();
};


#endif
