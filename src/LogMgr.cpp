#include "CompilationOpts.h"

#ifdef USE_LOGGING

#include "LogMgr.h"

void LogValue::formatInto(String *str)
{
    switch (type) {
        case VOID:
            str->concat("");
            break;
        case BOOL:
            if (format != nullptr) {
                char v[20];
                snprintf(v, sizeof(v), format, val.boolVal);
                str->concat(v);
            } else {
                str->concat(this->val.boolVal ? "Y" : "N");
            }
            break;
        case INT:
            if (format == nullptr) {
                str->concat((long)val.int64Val);
            } else {
                char v[20];
                snprintf(v, sizeof(v), format, val.int64Val);
                str->concat(v);
            }
            break;
        case UINT:
            if (format == nullptr) {
                str->concat((unsigned long)val.int64Val);
            } else {
                char v[20];
                snprintf(v, sizeof(v), format, val.int64Val);
                str->concat(v);
            }
            break;
        case FLOAT:
            if (format == nullptr) {
                str->concat(val.floatVal);
            } else {
                char v[20];
                snprintf(v, sizeof(v), format, val.floatVal);
                str->concat(v);
            }
            break;
        case DOUBLE:
            if (format == nullptr) {
                str->concat(val.doubleVal);
            } else {
                char v[20];
                snprintf(v, sizeof(v), format, val.doubleVal);
                str->concat(v);
            }
            break;
        case STR:
            {
                const char *s = (val.cstrVal == nullptr ? "<NULL>" : val.cstrVal);
                if (format == nullptr) {
                    str->concat(s);
                } else {
                    int n = strlen(s);
                    char v[20 + n];
                    snprintf(v, sizeof(v), format, s);
                    str->concat(v);
                }
            }
            break;
        case FN:
            {
                String s;
                val.fnVal(&s);
                if (format == nullptr) {
                    str->concat(s);
                } else {
                    char v[20 + s.length()];
                    snprintf(v, sizeof(v), format, s.c_str());
                    str->concat(v);
                }
            }
            break;
        default:
            break;
    }
}



LogMgr::LogMgr()
: purgeCondition(&purgeMonitor), purgePreservePos(LONG_MAX), loggers(0)
{
    records.capacity(511);
    values.capacity(1024);
    nextRecordPos = 1; // we're not using pos 0 at all, we're starting from 1. Can't use -1 for oldestRecordPosToFlush.
    oldestRecordPosToFlush = 0;
    globalLogLevel = LogLevel::LOGLVL_INFO;
    isSerialImmediate = false;
}

void LogMgr::init()
{

}

LogMgr::~LogMgr()
{

}

Logger *LogMgr::newLogger(const char *name)
{
    {
        MonitorScope mScope(&monitor);
        Logger *logger = new Logger(this, name, globalLogLevel);
        loggers.push_back(logger);
        return logger;
    } // MonitorScope
}

void LogMgr::deleteLogger(Logger *logger)
{
    {
        MonitorScope mScope(&monitor);
        for (auto i = loggers.begin(); i != loggers.end(); i++) {
            if (*i == logger) {
                loggers.erase(i);
                break;
            }
        }
        delete logger;
    } // MonitorScope
}

void LogMgr::doLog(const char *name, LogLevel level, const char *format, int valCount,
        const LogValue *val1, const LogValue *val2,
        const LogValue *val3, const LogValue *val4,
        const LogValue *val5, const LogValue *val6,
        const LogValue *val7, const LogValue *val8)
{
    if (globalLogLevel < level) {
        return;
    }

    if (format == nullptr) {
        format = "<No format was specified!!!>";
    }

    monitor.enter();

    int purgeCount = (records.capacity() + 7) / 8; // round up

    // purging
    while ((!values.canAddHead(valCount) && values.size() > 0) || !records.canAddHead(1)) {
        int entries = records.size();
        int toPop = (entries > purgeCount ? purgeCount : entries);
        int valuesToPop = 0;
        for (int i = 0; i < toPop; i++) {
            LogRecord *r = records.atTail(i);
            valuesToPop += r->valueCount;
        }

        // if another thread is reading records and wants to preserve records that we want to purge, wait
        while (purgePreservePos != ULONG_MAX && purgePreservePos <= nextRecordPos - records.size() + toPop) {
            purgeCondition.wait();
        }
// Serial.printf("    Have %d record, %d values, purging %d records and %d values\n", records.size(), values.size(), toPop, valuesToPop);
        for (int i = 0; i < toPop; i++) {
            records.atTail(i)->clear();
        }
        records.removeTail(toPop);
        for (int i = 0; i < valuesToPop; i++) {
            values.atTail(i)->clear();
        }
        values.removeTail(valuesToPop);
// Serial.printf("    Remaining %d records (%d..%d), %d values (%d..%d)\n",
//            records.size(), records.headIdx(), records.tailIdx(),
//            values.size(), values.headIdx(), values.tailIdx());
    }

    // We're setting a new record, verify it is "clean" (we clean it on constructor and after purging)
    LogRecord *rec = records.addHead();
    if (rec->name != nullptr) {
        Serial.printf("Setting log record over an existing record at head idx %d, existing name: %s\n", records.headIdx(), rec->name);
        abort();
    }
    if (!values.canAddHead(valCount)) {
        // we can't log this record because it has more values than what we can accomodate
        // log an entry with no values
        rec->set(name, level, format, values.nextIdx(), 0);
    } else {
        // log normally
        int startingIdx = values.nextIdx();
        values.addHead(valCount);
        // verify we're adding to empty values
        for (int i = 0; i < valCount; i++) {
            if (values.atHead((valCount - 1) - i)->type != LogValue::Type::VOID) {
                Serial.printf("We're adding a value to a non-empty value slot! Record value %d of %d, head index after adding: %d\n",
                    i, valCount, values.headIdx());
                abort();
            }
        }
        switch (valCount) { // dropping though at each case
            case 8: values.atHead((valCount - 1) - 7)->set(*val8);
            case 7: values.atHead((valCount - 1) - 6)->set(*val7);
            case 6: values.atHead((valCount - 1) - 5)->set(*val6);
            case 5: values.atHead((valCount - 1) - 4)->set(*val5);
            case 4: values.atHead((valCount - 1) - 3)->set(*val4);
            case 3: values.atHead((valCount - 1) - 2)->set(*val3);
            case 2: values.atHead((valCount - 1) - 1)->set(*val2);
            case 1: values.atHead((valCount - 1) - 0)->set(*val1);
        }
        rec->set(name, level, format, startingIdx, valCount);
    }
    ++nextRecordPos;
// Serial.printf("Leaving doLog(), records: %d .. %d, values: %d .. %d\n",
//         records.tailIdx(), records.headIdx(), values.tailIdx(), values.headIdx());

    if (isSerialImmediate) {
        String str;
        const LogValue* v[8];
        v[0] = val1; v[1] = val2; v[2] = val3; v[3] = val4;
        v[4] = val5; v[5] = val6; v[6] = val7; v[7] = val8;

        this->format(&str, format, valCount, const_cast<LogValue**>(v));
        Serial.printf("LOGGING: %s %lu %s %s\n", name, millis(), levelName(level), str.c_str());
    }

    monitor.leave();
}

uint64_t LogMgr::getFirstRecordIdx()
{
    monitor.enter();
    uint64_t idx = nextRecordPos - (uint64_t)records.size();
    monitor.leave();
    return idx;
}

uint64_t LogMgr::getLastRecordIdx()
{
    monitor.enter();
    uint64_t idx = nextRecordPos - 1;
    monitor.leave();
    return idx;
}

bool LogMgr::getRecord(uint64_t idx, String *name, uint32_t *timestamp, LogLevel *level, String *str)
{
    LogRecord rec;
    const int valMaxCount = 8;
    uint64_t headRelativeIdx;
    LogValue *val[valMaxCount]; // we'll handle up to 8 values;

    {
        MonitorScope ms(&monitor);
        while (purgePreservePos != LONG_MAX) {
            purgeCondition.wait(); // someone else is reading, we can't modify purgePreservePos
        }
        // we've reserved our record so it is not purged (nor its values)
        purgePreservePos = idx;

        headRelativeIdx = nextRecordPos - 1 - idx;
        if (headRelativeIdx < 0 || headRelativeIdx > INT_MAX) {
            return false;
        }
        LogRecord *r = records.atHead((int)headRelativeIdx);
        if (r == nullptr) {
            return false;
        }
        rec.set(*r);
        int vCnt = rec.valueCount > valMaxCount ? valMaxCount : rec.valueCount;
        for (int i = 0; i < vCnt; i++) {
            val[i] = values.at(rec.startingValueIdx + i);
        }
    } // monitor scope


    // now, out of monitor section, format values
    str->clear();
    if (rec.name == nullptr) {
        // this is a non-initialized record, this is an internal error
        str->concat("Non-initialized record at idx = "); str->concat((unsigned long)idx);
        str->concat(", nextRecordPos = "); str->concat((unsigned long)nextRecordPos);
        str->concat(", headRelativeIdx = "); str->concat((unsigned long)headRelativeIdx);
        str->concat(", records.size() = "); str->concat(records.size());
        *name = "";
        *timestamp = 0;
        *level = LogLevel::LOGLVL_FATAL;
    } else {
        format(str, rec.format, rec.valueCount < valMaxCount ? rec.valueCount : valMaxCount, val);
        *name = rec.name;
        *timestamp = rec.timestamp;
        *level = rec.level;
    }

    {
        MonitorScope ms(&monitor);
        purgePreservePos = LONG_MAX;
        purgeCondition.notifyAll();
    }

    return true;
}

void LogMgr::format(String *str, const char *format, int valueCount, LogValue **values)
{
    if (format == nullptr) {
        str->concat("<No format was given...>");
        return;
    }
    String tmp;
    const char *p = format;
    int valIdx = 0;
    while (*p != '\0') {
        if (p[0] == '{') {
            if (p[1] == '}') {
                if (valIdx < valueCount) {
                    tmp.clear();
                    values[valIdx]->formatInto(&tmp);
                    str->concat(tmp);
                    ++valIdx;
                } else {
                    str->concat("{}"); // there's no value for this placeholder
                }
                p += 2;
            } else if (p[1] != '\0') {
                str->concat(p[1]);
                p += 2;
            } else {
                ++p; // and we're at the end of the string
            }
        } else {
            str->concat(*p);
            ++p;
        }
    }
}

LogMgr::FlusherHandle LogMgr::addFlusher(FlushFunction flushFunction)
{
    MonitorScope msf(&flusherListMonitor);
    MonitorScope ms(&monitor);

    int idx = -1;
    for (int i = 0; i < flushers.size(); i++) {
        if (!flushers[i].isOccupied) {
            idx = i;
            break;
        }
    }
    if (idx == -1) {
        flushers.push_back(FlusherEntry());
        idx = flushers.size() - 1;
    }
    flushers[idx].isOccupied = true;
    flushers[idx].flusher = flushFunction;
    flushers[idx].recordPosToFlush = nextRecordPos - records.size();
    return idx;
}

void LogMgr::removeFlusher(FlusherHandle handle)
{
    MonitorScope msf(&flusherListMonitor);
    MonitorScope ms(&monitor);

    if (handle < 0 || handle > flushers.size()) {
        return; // bad handle
    }
    if (!flushers[handle].isOccupied) {
        return; // flusher has already been removed
    }
    flushers[handle].isOccupied = false;
    flushers[handle].flusher = nullptr;
}

void LogMgr::callFlushers()
{
    monitor.enter();
    uint64_t lastHeadPos = nextRecordPos;
    monitor.leave();

    if (lastHeadPos > oldestRecordPosToFlush) {
        oldestRecordPosToFlush = 0;

        MonitorScope msf(&flusherListMonitor);

        for (auto i = flushers.begin(); i != flushers.end(); i++) {
            if (!i->isOccupied) {
                continue;
            }
            i->recordPosToFlush = i->flusher(this, i->recordPosToFlush, lastHeadPos - i->recordPosToFlush);
            oldestRecordPosToFlush = (i->recordPosToFlush < oldestRecordPosToFlush ? i->recordPosToFlush : oldestRecordPosToFlush);
        }
    }
}

void LogMgr::setGlobalLevel(LogLevel level)
{
    globalLogLevel = level;
}

LogLevel LogMgr::getGlobalLevel()
{
    return globalLogLevel;
}

void LogMgr::setCapacity(int recordsCapacity, int valuesCapacity)
{
    // must clear before, otherwise records' indexes into values will be wrong
    clear();
    records.capacity(recordsCapacity);
    values.capacity(valuesCapacity);
}

int LogMgr::getRecordsCapacity()
{
    return records.capacity();
}

void LogMgr::clear()
{
    // must clear before, otherwise records' indexes into values will be wrong
    for (int i = 0; i < records.size(); i++) {
        records.atHead(i)->clear();
    }
    for (int i = 0; i < values.size(); i++) {
        values.atHead(i)->clear();
    }
    records.clear();
    values.clear();
}

void LogMgr::setSerialImmediate(bool enable)
{
    isSerialImmediate = enable;
}

const char *LogMgr::levelName(LogLevel level)
{
    switch (level) {
    case LOGLVL_OFF: return "OFF";
    case LOGLVL_FATAL: return "FATAL";
    case LOGLVL_ERROR: return "ERROR";
    case LOGLVL_WARN: return "WARN";
    case LOGLVL_INFO: return "INFO";
    case LOGLVL_DEBUG: return "DEBUG";
    case LOGLVL_TRACE: return "TRACE";
    case LOGLVL_ALL: return "ALL";
    default: return "";
    }
};


// LogService

LogService::LogService()
{
    isTesting = false;
    currentTestName = nullptr;
}

#ifdef LOGGING_ENABLE_TESTS

void LogService::checkRecord(uint64_t idx, const char *msgToCompare)
{
    String name;
    uint32_t timestamp;
    LogLevel level;
    String str;
Serial.printf("Checking record %" PRIu64 ", current records from %" PRIu64 " to %" PRIu64 "\n",
        idx, logMgr->getFirstRecordIdx(), logMgr->getLastRecordIdx());
    bool ok = logMgr->getRecord(idx, &name, &timestamp, &level, &str);
    if (ok) {
        Serial.printf("Got record [%" PRIu64 "] %s, level %d: %s\n", idx, name.c_str(), level, str.c_str());
        if (msgToCompare != nullptr && strcmp(msgToCompare, str.c_str()) != 0) {
            Serial.printf("    **** Message does not match with \"%s\"\n", msgToCompare);
        }
    } else {
        Serial.printf("    **** getRecord(%lld) returned false\n", idx);
    }

Serial.printf("Getting first record %" PRIu64 "\n", logMgr->getFirstRecordIdx());
logMgr->getRecord(logMgr->getLastRecordIdx(), &name, &timestamp, &level, &str);
Serial.printf("Got first record %" PRIu64 "\n", logMgr->getFirstRecordIdx());
Serial.printf("Getting last record %" PRIu64 "\n", logMgr->getLastRecordIdx());
logMgr->getRecord(logMgr->getFirstRecordIdx(), &name, &timestamp, &level, &str);
Serial.printf("Got last record %" PRIu64 "\n", logMgr->getLastRecordIdx());

    Serial.flush();
};

void LogService::testStart(const char *testName)
{
    currentTestName = testName;
    Serial.printf("Starting test %s, logMgr records: %" PRIu64 " .. %" PRIu64 "\n", testName,
        logMgr->getFirstRecordIdx(), logMgr->getLastRecordIdx());
    Serial.flush();
};

void LogService::testEnd()
{
    Serial.printf("Ending test %s, logMgr records: %" PRIu64 " .. %" PRIu64 "\n", currentTestName,
        logMgr->getFirstRecordIdx(), logMgr->getLastRecordIdx());
    currentTestName = nullptr;
    Serial.flush();
};

#endif

void LogService::init(LogMgr *logMgr, UEventLoop *eventLoop, CommandMgr *commandMgr)
{
    this->logMgr = logMgr;
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("LogService");
    eventLoop->registerTimer(&logFlusherTimer);
    eventLoop->registerTimer(&testTimer);

    syslogServer = "";
    syslogPort = 514;

    ServiceCommands *cmd = commandMgr->getServiceCommands("logger");

    cmd->onAfterStatus([this](String *msg) {
        char buf[200];
        snprintf(buf, sizeof(buf), "Logs from %" PRIu64 " to %" PRIu64 "\n",
            this->logMgr->getFirstRecordIdx(), this->logMgr->getLastRecordIdx());
        buf[sizeof(buf) - 1] = '\0';
        msg->concat(buf);
        snprintf(buf, sizeof(buf), "Log buffer: %d lines\n", this->logMgr->getRecordsCapacity());
        buf[sizeof(buf) - 1] = '\0';
        msg->concat(buf);
    });

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("recordsCapacity", true)
        .cmd("recordsCapacity")
        .help("--> Number of records in the log")
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            this->logMgr->setCapacity(val, val * 2);
            *msg = "Capacity set to "; *msg += val; *msg += " records, and twice that for values. Log cleared.";
            return true;
        })
        .getFn([this]() {
            return this->logMgr->getRecordsCapacity();
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("serialImmediate", true)
        .cmd("serialImmediate")
        .help("--> If set, a log line is immediately printed into Serial.")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            this->logMgr->setSerialImmediate(val);
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("level", true)
        .cmd("level")
        .help("--> Logging level: OFF, FATAL, ERROR, WARN, INFO, DEBUG, TRACE, ALL")
        .setFn([this](const String &val, bool isLoading, String *msg) -> bool {
            LogLevel level;
            if (strcasecmp(val.c_str(), "ALL") == 0) {
                level = LogLevel::LOGLVL_ALL;
            } else if (strcasecmp(val.c_str(), "FATAL") == 0) {
                level = LogLevel::LOGLVL_FATAL;
            } else if (strcasecmp(val.c_str(), "ERROR") == 0) {
                level = LogLevel::LOGLVL_ERROR;
            } else if (strcasecmp(val.c_str(), "WARN") == 0) {
                level = LogLevel::LOGLVL_WARN;
            } else if (strcasecmp(val.c_str(), "INFO") == 0) {
                level = LogLevel::LOGLVL_INFO;
            } else if (strcasecmp(val.c_str(), "DEBUG") == 0) {
                level = LogLevel::LOGLVL_DEBUG;
            } else if (strcasecmp(val.c_str(), "TRACE") == 0) {
                level = LogLevel::LOGLVL_TRACE;
            } else if (strcasecmp(val.c_str(), "ALL") == 0) {
                level = LogLevel::LOGLVL_ALL;
            } else {
                *msg = "Unrecognized level \"";
                *msg += val;
                *msg += "\"";
                return true;
            }
            this->logMgr->setGlobalLevel(level);
            *msg = "Global log level set to "; *msg += this->logMgr->levelName(level);
            return true;
        })
        .getFn([this](String *val) {
            *val = this->logMgr->levelName(this->logMgr->getGlobalLevel());
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("log", true)
        .cmd("log")
        .isPersistent(false)
        .help("--> Send a line to the log")
        .setFn([this](const String &val, bool isLoading, String *msg) -> bool {
            this->logger->info("{}", val.c_str());
            *msg = "Sent to log: "; *msg += val;
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("clear", true)
        .cmdOn("clear")
        .isPersistent(false)
        .helpOn("--> Clear all log lines")
        .setFn([this](bool val, bool isLoading, String *msg) -> bool {
            this->logMgr->clear();
            *msg = "Log cleared";
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("last", true)
        .cmd("last")
        .isPersistent(false)
        .help("--> last <n>: List last <n> log lines. Use -1 to list from the first available log line.")
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            getLast(val, msg, LogLevel::LOGLVL_ALL);
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("lastErrors", true)
        .cmd("lastErrors")
        .isPersistent(false)
        .help("--> lastError <n>: List last <n> log lines, show only lines in ERROR. Use -1 to list from the first available log line.")
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            getLast(val, msg, LogLevel::LOGLVL_ERROR);
            return true;
        })
    );

#ifdef LOGGING_USE_SYSLOG
    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("syslogServer", true)
        .cmd("syslogServer")
        .help("--> Syslog server name or IP address, or \"off\" to turn off.")
        .getFn([this](String *val) {
            if (syslogServer.isEmpty()) {
                *val = "off";
            } else {
                *val = syslogServer;
            }
        })
        .setFn([this](const String &val, bool isLoading, String *msg) {
            if (val == "off") {
                syslogServer = "";
                *msg = "syslogServer set to "; msg->concat(syslogServer);
            } else {
                syslogServer = val;
                *msg = "syslogServer off";
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("syslogPort", true)
        .cmd("syslogPort")
        .help("--> Port of the syslog server. Requires save and reboot.")
        .ptr(&syslogPort)
    );

#endif

#ifdef LOGGING_ENABLE_TESTS
    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("test", true)
        .cmd("test")
        .isPersistent(false)
        .help("--> Run tests. Please use \"log recordsCapacity\" beforehand, setting capacity to around 20")
        .setFn([this](const String &val, bool isLoading, String *msg) -> bool {
            if (isTesting) {
                *msg = "A test is already in progress: ";
                msg->concat(currentTestName != nullptr ? currentTestName : "<unknown>");
                return true;
            }
            isTesting = true;

            LogMgr *logMgr = this->logMgr;
            logMgr->setGlobalLevel(LogLevel::LOGLVL_ALL);

            Logger *logger = logMgr->newLogger("myLogger");
            logger->setLevel(LogLevel::LOGLVL_ALL);

            testStart("Test 0 - noop");
            testEnd();

            testStart("Test 1");
            logger->error("A debug message: {}", 100);
            Serial.printf("Message 1 sent, logMgr idx: [%d .. %d]\n",
                (int)logMgr->getFirstRecordIdx(),
                (int)logMgr->getLastRecordIdx());
            checkRecord(logMgr->getLastRecordIdx(), "A debug message: 100");
            testEnd();

            testStart("Test 2");
            logger->debug("A debug message: {}", 200);
            checkRecord(logMgr->getLastRecordIdx() - 1, "A debug message: 100");
            checkRecord(logMgr->getLastRecordIdx(), "A debug message: 200");
            testEnd();

            testStart("Test 3");
            logger->debug("A debug message: {}/{}", 100, (uint16_t)200);
            checkRecord(logMgr->getLastRecordIdx() - 2, "A debug message: 100");
            checkRecord(logMgr->getLastRecordIdx() - 1, "A debug message: 200");
            checkRecord(logMgr->getLastRecordIdx(), "A debug message: 100/200");
            testEnd();

            testStart("Test 4");
            logger->debug("A debug message: {}/{}", "abc", String("def").c_str());
            checkRecord(logMgr->getLastRecordIdx(), "A debug message: abc/def");
            testEnd();

            testStart("Test 5");
            logger->debug("A debug message: {}/{}", "abc", "ghi with copy");
            checkRecord(logMgr->getLastRecordIdx() - 1, "A debug message: abc/def");
            checkRecord(logMgr->getLastRecordIdx(), "A debug message: abc/ghi with copy");
            testEnd();

            testStart("Test 7");
            std::function<void (String *)> fn = [](String *str) -> void { str->concat("from function"); };
            logger->debug("A message: {}", fn);
            checkRecord(logMgr->getLastRecordIdx(), "A message: from function");
            testEnd();

            // continue in a timer

            struct T8 {
                int total;
                int step;
                int next;
                std::function<void(UEventLoopTimer * timer)> nextFn;
            };
            T8 *t8 = new T8();
            // send many lines to test purging and queue
            t8->total = 40;
            t8->step = 5;
            t8->next = 0;
            t8->nextFn = nullptr;
            testStart("Test 8");
            testTimer.setTimeout([this, logMgr, logger, t8](UEventLoopTimer * timer) {
                int n = t8->next + t8->step;
                if (n > t8->total) {
                    n = t8->total;
                }
                for (int i = t8->next; i < n; i++) {
                    logger->debug("Message {}", i);
                }
                if (n < t8->total) {
                    t8->next = n;
                    Serial.printf("---> Sent up to %d of %d, resetting the timeout\n", t8->next, t8->total);
                    Serial.flush();
                    timer->setTimeout(10); // keeping the same timer callback
                } else { // we're terminating
                    Serial.printf("---> Terminating, next=%d, total=%d\n", t8->next, t8->total);
                    Serial.flush();
                    this->checkRecord(logMgr->getLastRecordIdx(), "Message 39");
                    this->testEnd();
                    if (t8->nextFn != nullptr) {
                        timer->setTimeout(t8->nextFn, 10);
                    }
                    delete t8;
                }
            }, 10);

            struct T9 {
                int total;
                int step;
                int next;
                std::function<void(UEventLoopTimer * timer)> nextFn;
            };
            T9 *t9 = new T9();
            t9->total = 20;
            t9->step = 5;
            t9->next = 0;
            t9->nextFn = nullptr;

            t8->nextFn = [this, t9, logger](UEventLoopTimer * timer) {
                if (t9->next == 0) {
                    testStart("Test 9");
                }
                int n = t9->next + t9->step;
                if (n > t9->total) {
                    n = t9->total;
                }
                for (int i = t9->next; i < n; i++) {
                   logger->debug("{} {} of {}", LogValue([](String *str) { str->concat("FnMessage"); }), i, t9->total);
                }
                if (n < t9->total) {
                    t9->next = n;
                    Serial.printf("---> Sent up to %d of %d, resetting the timeout\n", t9->next, t9->total);
                    Serial.flush();
                    timer->setTimeout(10); // keeping the same timer callback
                } else { // we're terminating
                    Serial.printf("---> Terminating, next=%d, total=%d\n", t9->next, t9->total);
                    Serial.flush();
                    this->checkRecord(this->logMgr->getLastRecordIdx(), "FnMessage 19 of 20");
                    this->testEnd();
                    if (t9->nextFn != nullptr) {
                        timer->setTimeout(t9->nextFn, 10);
                    }
                    delete t9;
                }
            };

            t9->nextFn = [this, logger](UEventLoopTimer * timer) {
                Serial.println("---> Terminating all tests");
                Serial.printf("Records: %" PRIu64 " .. %" PRIu64 "\n", this->logMgr->getFirstRecordIdx(), this->logMgr->getLastRecordIdx());
                Serial.flush();
                Serial.println("---> Deleting logger");
                Serial.flush();
                this->logMgr->deleteLogger(logger);
                isTesting = false;
            };



#if 0
            LogMgr::FlusherHandle flusherHandle = logMgr->addFlusher([](LogMgr *logMgr, int flushFrom, int count) {
                String name;
                LogLevel level;
                String str;
                for (uint64_t i = flushFrom; i < flushFrom + count; i++) {
                    bool gotLine = logMgr->getRecord(i, &name, &level, &str);
                    if (gotLine) {
                        Serial.println(str);
                        str.clear();
                    }
                }
                return flushFrom + count - 1;
            });

            logMgr->removeFlusher(flusherHandle);
#endif

            *msg = "Tests were spawned";
            return true;
        })
    );

#endif // LOGGING_ENABLE_TESTS

    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

#ifdef LOGGING_USE_SYSLOG
    if (!syslogServer.isEmpty()) {
        logMgr->addFlusher([this](LogMgr *logMgr, uint64_t flushFrom, int count) {
            if (syslogServer.isEmpty() || syslogPort == 0) {
                return flushFrom + count;
            }

            String name;
            uint32_t timestamp;
            LogLevel level;
            String str;
            for (uint64_t i = flushFrom; i < flushFrom + count; i++) {
                bool found = logMgr->getRecord(i, &name, &timestamp, &level, &str);
                if (found) {
                    Serial.printf("LOG FLUSHER: [%" PRIu64 "] %d %s %s %s\n", i, timestamp, name.c_str(), logMgr->levelName(level), str.c_str());
                }
            }
            return flushFrom + count;
        });
    }
#endif

    logFlusherTimer.setCallback([this](UEventLoopTimer *timer) {
        this->logMgr->callFlushers();
    });
    logFlusherTimer.setInterval(50); // 20 times/second

}

void LogService::getLast(int val, String *msg, int maxLevel)
{
    uint64_t last = this->logMgr->getLastRecordIdx();
    uint64_t first;
    if (val <= 0) {
        first = this->logMgr->getFirstRecordIdx();
    } else {
        first = last - val + 1;
        uint64_t f = this->logMgr->getFirstRecordIdx();
        if (first < f) {
            first = f;
        }
    }
    *msg = "";
    String str;
    String name;
    LogLevel level;
    uint32_t timestamp;
    msg->concat("Log lines from " + String((int)first) + " to " + (int)last + "\n");
    for (uint64_t i = first; i <= last; i++) {
        str.clear();
        bool ok = this->logMgr->getRecord(i, &name, &timestamp, &level, &str);
        if (ok && level > maxLevel) {
            continue;
        }
        msg->concat((uint32_t)i);
        msg->concat(' ');
        if (ok) {
            if (timestamp > 1000 * 60 * 60) {
                uint32_t h = timestamp / 3600000;
                msg->concat(h);
                msg->concat(':');
                timestamp -= h * 3600000;
            }
            uint32_t m = timestamp / 60000;
            if (m < 10) {
                msg->concat('0');
            }
            msg->concat(m);
            msg->concat(':');
            timestamp -= m * 60000;
            uint32_t s = timestamp / 1000;
            if (s < 10) {
                msg->concat('0');
            }
            msg->concat(s);
            msg->concat('.');
            timestamp -= s * 1000;
            if (timestamp < 10) {
                msg->concat("00");
            } else if (timestamp < 100) {
                msg->concat("0");
            }
            msg->concat(timestamp);
            msg->concat(": "); msg->concat(name); msg->concat(" ");
            msg->concat(this->logMgr->levelName(level)); msg->concat(" ");
            msg->concat(str);
            msg->concat("\n");
        } else {
            msg->concat(": <nonexistent>\n");
        }
    }
}


#endif
