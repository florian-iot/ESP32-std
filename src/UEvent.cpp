#include <HardwareSerial.h>
#include <WString.h>
#include <functional>
#include <FreeRTOS.h>
#include <freertos/queue.h>
#include "UEvent.h"


UEventLoop::UEventLoop(const char *loopName)
{
    this->loopName = loopName;
    queue = xQueueCreate(10, sizeof(UEventEntry));
    // after this point, queue is considered read-only, safe to read from all threads
    processorSequence = 0;
    isShuttingDown.store(false);
    nextEventClassType = 1;
    eventClasses.push_back(nullptr);
    nextEventNameType = 1;
    eventNames.push_back(EventTypeEntry(nullptr, nullptr, 0));
    loopTask = nullptr;
    scheduler.init();
}

UEventLoop::~UEventLoop()
{
    vQueueDelete(queue);
    // eventNames[*].eventClass == eventClasses[*], we allocate eventClass only once
    for (int i = 1; i < eventNames.size(); i++) {
        free(eventNames[i].eventClass);
        free(eventNames[i].eventName);
        eventNames[i].eventClass = nullptr;
        eventNames[i].eventName = nullptr;
        eventClasses[i] = nullptr;
    }
}

uint32_t UEventLoop::getEventClassType(const char *eventClass)
{
    mon.enter();
    uint32_t eventClassType = getEventClassTypeInternal(eventClass);
    mon.leave();
    return eventClassType;
}

// always called with mon entered
uint32_t UEventLoop::getEventClassTypeInternal(const char *eventClass)
{
    uint16_t eventClassType = 0;
    for (int i = 1; i < eventClasses.size(); i++) {
        if (strcmp(eventClasses[i], eventClass) == 0) {
            eventClassType = (uint16_t)i;
            break;
        }
    }
    if (eventClassType == 0) {
        eventClassType = nextEventClassType;
        char *internalEventClass = strdup(eventClass);
        eventClasses.push_back(internalEventClass);
        ++nextEventClassType;
    }
    uint32_t eventType = ((uint32_t)eventClassType) << 16;
    return eventType;
}

uint32_t UEventLoop::getEventType(const char *eventClass, const char *eventName)
{
    mon.enter();
    uint32_t eventType = 0;
    for (uint16_t i = 1; i < eventNames.size(); i++) {
        if (strcmp(eventNames[i].eventClass, eventClass) == 0
                && strcmp(eventNames[i].eventName, eventName) == 0) {
            eventType = eventNames[i].eventType;
            break;
        }
    }
    if (eventType == 0) { // not found
        uint32_t eventClassType = getEventClassTypeInternal(eventClass);
        char *internalEventName = strdup(eventName);
        eventType = eventClassType + nextEventNameType;
        EventTypeEntry eventTypeEntry(eventClasses[eventClassType >> 16], internalEventName, eventType);
        eventNames.push_back(eventTypeEntry);
        ++nextEventNameType;
    }
    mon.leave();
    return eventType;
}

uint32_t UEventLoop::findEventClassType(const char *eventClass)
{
    uint32_t eventType = 0;
    mon.enter();
    for (uint16_t i = 1; i < eventClasses.size(); i++) {
        if (strcmp(eventClasses[i], eventClass) == 0) {
            eventType = ((uint32_t)i) << 16;
            break;
        }
    }
    mon.leave();
    return eventType;
}

uint32_t UEventLoop::findEventType(const char *eventClass, const char *eventName)
{
    uint32_t eventType = 0;
    mon.enter();
    for (uint16_t i = 1; i < eventNames.size(); i++) {
        if (strcmp(eventNames[i].eventClass, eventClass) == 0
                && strcmp(eventNames[i].eventName, eventName) == 0) {
            eventType = eventNames[i].eventType;
            break;
        }
    }
    mon.leave();
    return eventType;
}

const char *UEventLoop::getEventClass(uint32_t eventType)
{
    int t = (eventType >> 16);
    if (t >= nextEventClassType) {
        return "?";
    }
    return eventClasses[eventType >> 16];
}

const char *UEventLoop::getEventName(uint32_t eventType)
{
    uint16_t t = eventType & 0xFFFF;
    if (t >= nextEventNameType) {
        return "?";
    }
    return (t == 0 ? nullptr : eventNames[t].eventName);
}

void UEventLoop::run()
{
    if (queue == nullptr) {
        return;
    }
    loopTask = xTaskGetCurrentTaskHandle();
    do {
        runOnce(100 / portTICK_PERIOD_MS); // every 100 millis if there's nothing to do, so we can shut down if needed in 100 ms
    } while (!isShuttingDown.load());
}

boolean UEventLoop::runOnce(long waitTicks)
{
#ifdef USE_EVENT_CHECK_HEAP
    if (!heap_caps_check_integrity_all(true)) {
        Serial.printf("Heap integrity failure at the beginning of event loop \"%s\"", loopName);
    }
#endif
    // run any scheduled tasks
    bool didExecutions = !scheduler.execute();
    long nextSchedulerIteration = scheduler.timeUntilNextIteration();
#ifdef USE_EVENT_CHECK_HEAP
    if (!heap_caps_check_integrity_all(true)) {
        Serial.printf("Heap integrity failure after scheduler tasks in event loop \"%s\"", loopName);
    }
#endif

    // process any messages
    if (queue == nullptr) {
        return didExecutions;
    }
    uint32_t startTick = xTaskGetTickCount();
    if (nextSchedulerIteration != -1) {
        uint32_t w = nextSchedulerIteration / (1000 * portTICK_PERIOD_MS);
        if (w < waitTicks) {
            waitTicks = w; // take the earliest time between waitUntilTick and w
        }
    }

    UEventEntry eventEntry;
    BaseType_t didReceive;
    int count = 0;
    long w = waitTicks;
    do {
        if (count >= 10) {
            break;
        }
        didReceive = xQueueReceive(queue, &eventEntry.event, w);
        if (didReceive != pdPASS) {
            break;
        }
        handleEvent(&eventEntry);
        uint32_t currentTick = xTaskGetTickCount();
        w = startTick + waitTicks - currentTick;
        if (w < 0) {
            w = 0;
        }
        ++count;
#ifdef USE_EVENT_CHECK_HEAP
        if (!heap_caps_check_integrity_all(true)) {
            Serial.printf("Heap integrity failure after processing event %s:%s in event loop \"%s\"",
                this->getEventClass(eventEntry.event.eventType),
                this->getEventName(eventEntry.event.eventType),
                loopName);
        }
#endif
    } while (true);

    if (!didExecutions && count == 0) { // if we did nothing, or if too long without a yield, do a yield, so that lower priority tasks run
        taskYIELD();
    }

    return (didExecutions || count > 0);
}

xTaskHandle UEventLoop::getProcessingTask()
{
    return loopTask;
}

bool UEventLoop::ProcessorEntry::matchesType(uint32_t inEventType)
{
    if ((eventType & 0xFFFF) == 0) { // class only
        return eventType == (inEventType & 0xFFFF0000);
    } else {
        return eventType == inEventType;
    }
}

void UEventLoop::handleEvent(UEventEntry *eventEntry) {
    // Serial.printf("Processing event of type %s:%s\n",
    //     getEventClass(eventEntry->event.eventType),
    //     getEventName(eventEntry->event.eventType)
    // );

    mon.enter();
    {
        bool didProcess = false;
        for (int i = 0; i < processors.size() && !didProcess; i++) {
            if (processors[i].matchesType(eventEntry->event.eventType)) {
                didProcess = processors[i].processor(&eventEntry->event);
            }
            eventEntry->isProcessed = didProcess;
        }
    }
    mon.leave();

    if (eventEntry->onProcess != nullptr) {
        eventEntry->onProcess(&eventEntry->event, eventEntry->isProcessed);
    }
    if (eventEntry->sem != nullptr) {
        xSemaphoreGive(eventEntry->sem);
    }
    if (eventEntry->finalizer) {
        eventEntry->finalizer(&eventEntry->event);
    }

    // Serial.printf("Done processing event of type %s:%s\n",
    //     getEventClass(eventEntry->event.eventType),
    //     getEventName(eventEntry->event.eventType)
    // );
}

UEventHandle_t UEventLoop::onEvent(uint32_t eventType, std::function<bool(UEvent *event)> processor)
{
    int id;
    mon.enter();
    {
        ProcessorEntry e;
        e.eventType = eventType;
        e.processor = processor;
        id = ++processorSequence;
        e.id = id;
        processors.push_back(e);
    }
    mon.leave();
    return UEventHandle_t(id);
}

void UEventLoop::unregister(UEventHandle_t eventHandler)
{
    mon.enter();
    {
        for (auto i = processors.begin(); i < processors.end(); i++) {
            if (i->id == eventHandler.handle) {
                processors.erase(i);
                break;
            }
        }
    }
    mon.leave();
}

bool UEventLoop::processEvent(UEvent event)
{
    Serial.printf("Processing event of type [%d:%d] %s:%s\n",
        event.eventType >> 16, event.eventType & 0xFFFF,
        getEventClass(event.eventType),
        getEventName(event.eventType)
    );

    UEventEntry eventEntry;
    eventEntry.event = event;
    eventEntry.sem = nullptr;
    eventEntry.onProcess = nullptr;

    handleEvent(&eventEntry);
    return eventEntry.isProcessed;
}

bool UEventLoop::queueEvent(UEvent &event, std::function<void(UEvent*)> finalizer, SemaphoreHandle_t sem)
{
    return queueEvent(event, finalizer, nullptr, sem);
}

bool UEventLoop::queueEvent(UEvent &event, std::function<void(UEvent*)> finalizer,
        std::function<void(UEvent *event, bool isProcessed)> onProcess, SemaphoreHandle_t sem)
{
    if (queue == nullptr) {
        return false;
    }

    // Serial.printf("Queuing event of type [%d:%d] %s:%s\n",
    //     event.eventType >> 16, event.eventType & 0xFFFF,
    //     getEventClass(event.eventType),
    //     getEventName(event.eventType)
    // );

    UEventEntry eventEntry;
    eventEntry.event = event;
    eventEntry.sem = sem;
    eventEntry.onProcess = onProcess;
    eventEntry.finalizer = finalizer;
    BaseType_t didSent = xQueueSend(queue, &eventEntry, pdMS_TO_TICKS(50));
    if (didSent != pdPASS) {
        if (eventEntry.onProcess != nullptr) {
            eventEntry.onProcess(&eventEntry.event, false);
        }
        if (sem != nullptr) {
           xSemaphoreGive(sem);
        }
        if (finalizer) {
            finalizer(&event);
        }
        return false;
    } else {
        return true;
    }
}

void UEventLoop::registerTimer(UEventLoopTimer *timer)
{
    timer->init(this, nullptr);
}


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


Esp32Timer::Esp32Timer(UEventLoop *eventLoop, const char *name) {
    this->eventLoop = eventLoop;
    this->name = name;
    handle = nullptr;
    callback = nullptr;
}

Esp32Timer::~Esp32Timer() {
    if (handle != nullptr) {
        esp_timer_stop(handle);
        esp_timer_delete(handle);
        handle = nullptr;
    }
}

/** May not be called when the callback is active */
void Esp32Timer::setCallback(std::function<void (Esp32Timer *timer, uint64_t expectedTime)> callback) {
    if (handle != nullptr) {
        esp_timer_stop(handle);
        esp_timer_delete(handle);
    }
    this->callback = callback;

    esp_timer_create_args_t cfg;
    cfg.callback = esp32TimerCallback;
    cfg.arg = this;
    cfg.name = name;
    esp_err_t rc = esp_timer_create(&cfg, &handle);
    if (rc != ESP_OK) {
        handle = nullptr;
    }
}

void Esp32Timer::setTimeoutMicros(uint64_t timeout) {
    if (handle != nullptr) {
        esp_timer_stop(handle);
        nextRun = esp_timer_get_time() + (int64_t)timeout;
        auto rc = esp_timer_start_once(handle, timeout);
        if (rc != ESP_OK) {
            Serial.printf("Failed Esp32Timer::setTimeoutMicros(): %d, for timeout = %ld\n", rc, (long)timeout);
        }
    }
}

void Esp32Timer::clearTimeout() {
    if (handle != nullptr) {
        esp_timer_stop(handle);
    }
}

void Esp32Timer::setIntervalMicros(uint64_t timeout) {
    if (handle != nullptr) {
        interval = timeout;
        esp_timer_stop(handle);
        nextRun = esp_timer_get_time() + timeout;
        esp_timer_start_periodic(handle, timeout);
    }
}

void Esp32Timer::clearInterval() {
    if (handle != nullptr) {
        esp_timer_stop(handle);
    }
}

void Esp32Timer::esp32TimerCallback(void *arg)
{
    Esp32Timer *t = (Esp32Timer *)arg;
    if (t->callback != nullptr) {
        t->callback(t, t->nextRun);
        t->nextRun += (uint64_t)t->interval; // useful only if we're in a setInterval(), else on next setXXX() we'll set nextRun.
    }
}

uint64_t Esp32Timer::currentTime()
{
    return esp_timer_get_time();
}

//
//************************
//

UEventLoopTimer::UEventLoopTimer(UEventLoop *eventLoop, std::function<void(UEventLoopTimer *)> callback)
:   eventLoop(eventLoop), schedulerTask(), callback(callback),
    scheduledCallback([this]() {
        if (this->callback) {
            this->callback(this);
        }
        if (schedulerTask.isLastIteration()) {
            schedulerTask.disable();
        }
    })
{
    if (eventLoop != nullptr) {
        eventLoop->scheduler.addTask(schedulerTask);
    }
    schedulerTask.set(1, -1, scheduledCallback);
    schedulerTask.disable();
}

UEventLoopTimer::~UEventLoopTimer()
{
    schedulerTask.disable();
    if (eventLoop != nullptr) {
        eventLoop->scheduler.deleteTask(schedulerTask);
    }
}

void UEventLoopTimer::init(UEventLoop *eventLoop, std::function<void(UEventLoopTimer *)> callback)
{
    if (this->eventLoop != nullptr) {
        Serial.println("On UEventLoopTimer::init(), eventLoop has already been initialized");
        abort();
    }
    this->eventLoop = eventLoop;
    this->callback = callback;
    eventLoop->scheduler.addTask(schedulerTask);
    schedulerTask.disable();
    schedulerTask.set(1, -1, scheduledCallback);
}

UEventLoop *UEventLoopTimer::getEventLoop()
{
    return eventLoop;
}

void UEventLoopTimer::setCallback(std::function<void(UEventLoopTimer *)> callback)
{
    this->callback = callback;
}

void UEventLoopTimer::setInterval(std::function<void(UEventLoopTimer *)> callback, long intervalMillis)
{
    this->callback = callback;
    schedulerTask.set(intervalMillis * 1000, -1, scheduledCallback);
    schedulerTask.enableIfNot();
}

void UEventLoopTimer::setInterval(long intervalMillis)
{
    schedulerTask.set(intervalMillis * 1000, -1, scheduledCallback);
    schedulerTask.enableIfNot();
}

void UEventLoopTimer::setIntervalMicros(std::function<void(UEventLoopTimer *)> callback, long intervalMicros)
{
    this->callback = callback;
    schedulerTask.set(intervalMicros, -1, scheduledCallback);
    schedulerTask.enableIfNot();
}

void UEventLoopTimer::setIntervalMicros(long intervalMicros)
{
    schedulerTask.set(intervalMicros, -1, scheduledCallback);
    schedulerTask.enableIfNot();
}

void UEventLoopTimer::cancelInterval()
{
    schedulerTask.disable();
}

void UEventLoopTimer::setTimeout(std::function<void(UEventLoopTimer *)> callback, long intervalMillis)
{
    this->callback = callback;
    schedulerTask.set(intervalMillis * 1000, 1, scheduledCallback);
    schedulerTask.enableIfNot();
}

void UEventLoopTimer::setTimeout(long intervalMillis)
{
    schedulerTask.set(intervalMillis * 1000, 1, scheduledCallback);
    schedulerTask.enableIfNot();
}

void UEventLoopTimer::setTimeoutMicros(std::function<void(UEventLoopTimer *)> callback, long intervalMicros)
{
    this->callback = callback;
    schedulerTask.set(intervalMicros, 1, scheduledCallback);
    schedulerTask.enableIfNot();
}

void UEventLoopTimer::setTimeoutMicros(long intervalMicros)
{
    schedulerTask.set(intervalMicros, 1, scheduledCallback);
    schedulerTask.enableIfNot();
}

void UEventLoopTimer::cancelTimeout()
{
    schedulerTask.disable();
}

bool UEventLoopTimer::isActive()
{
    return schedulerTask.isEnabled();
}

long UEventLoopTimer::getOverrunMicros()
{
    return schedulerTask.getOverrun();
}

void UEventLoopTimer::unregister()
{
    schedulerTask.disable();
    eventLoop->scheduler.deleteTask(schedulerTask);
}
