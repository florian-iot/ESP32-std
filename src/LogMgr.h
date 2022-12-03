#ifndef LOGMGR_H
#define LOGMGR_H

#include "CompilationOpts.h"
#ifdef USE_LOGGING

#include <stdint.h>
#include <vector>
#include <functional>
#include <WString.h>

#include "Monitor.h"
#include "UEvent.h"
#include "CommandMgr.h"

class LogMgr;
class Logger;

enum LogLevel {
    LOGLVL_OFF,
    LOGLVL_FATAL,
    LOGLVL_ERROR,
    LOGLVL_WARN,
    LOGLVL_INFO,
    LOGLVL_DEBUG,
    LOGLVL_TRACE,
    LOGLVL_ALL
};

class LogValue {
    friend class LogMgr;

    union Val {
        bool boolVal;
        int64_t int64Val;
        float floatVal;
        double doubleVal;
        const char *cstrVal;
        std::function<void (String *)> fnVal;

        Val() : fnVal(nullptr) { }
        ~Val() { fnVal = nullptr; }
    } val;
    enum Type {
        VOID,
        BOOL,
        INT,
        UINT,
        FLOAT,
        DOUBLE,
        STR,
        FN
    };
    Type type;
    const char *format;

public:
    LogValue();
    ~LogValue();
    LogValue(int val, const char *format = nullptr);
    LogValue(unsigned int val, const char *format = nullptr);
    LogValue(int64_t val, const char *format = nullptr);
    LogValue(uint64_t val, const char *format = nullptr);
    LogValue(double val, const char *format = nullptr);
    LogValue(const char *val, const char *format = nullptr);
    typedef std::function<void (String *)> ValueFunction;
    LogValue(ValueFunction val, const char *format = nullptr);

    // moves ownership of strings-to-be-freed to this
    void set(const LogValue &val);
    void clear();
    void formatInto(String *str);

    // moves ownership of strings-to-be-freed to this
    LogValue(const LogValue &other); // for copied strings, "other" will lose the value (will get null instead)

private:
    LogValue &operator=(const LogValue &other) = delete;
};

class LogRecord {
friend class LogMgr;
    const char *name; // not allocated
    uint32_t timestamp;
    LogLevel level;
    const char *format; // not allocated
    int startingValueIdx; // index in LogMgr::values, where we serialize all values
    int valueCount;

    LogRecord &operator=(const LogRecord &other) = delete; // use explicit set()
    LogRecord(const LogRecord &other) = delete; // use explicit LogRecord() and set()
public:
    LogRecord();
    void set(const char *name, LogLevel level, const char *format, int startingValueIdx, int valueCount);
    void set(const LogRecord &other);
    void clear();
};


template < class T >
class Queue {
    T *buf;
    int length;
    uint64_t headPosition;
    int _nextIdx; // next position where an element will be added, [0..length-1]
    int _tailIdx; // position of oldest element
public:
    Queue();
    ~Queue();
    void capacity(int n);
    int capacity();
    int size();
    int headIdx();
    int nextIdx();
    int tailIdx();
    bool canAddHead();
    bool canAddHead(int n);
    T *addHead();
    void addHead(int n);
    T *at(int idx); // returns null if idx is not in range [_tailIdx .. _headIdx - 1] (% length)
    T *atHead();
    T *atHead(int i); // i-th element from the head
    T *atTail();
    T *atTail(int i); // i-th element from the tail, tail(0) == tail()
    bool canRemoveTail();
    bool canRemoveTail(int n);
    void removeTail();
    void removeTail(int n);
    void clear();
};

// Multithread-safe
class LogMgr {
    Monitor monitor;
    Monitor purgeMonitor;
    Condition purgeCondition;
    uint64_t purgePreservePos; // used during getRecord(), to preserve from any purges the record that is being read
    std::vector<Logger*> loggers;
    Queue<LogRecord> records;
    Queue<LogValue> values;
public:
    typedef std::function<uint64_t (LogMgr *logMgr, uint64_t flushFrom, int count)> FlushFunction;
    // the flusher function returns the index of the next record it will want to flush
    // (normally, flushFrom + count, but could be flushFrom if nothing was flushed)
    typedef int FlusherHandle;
private:
    Monitor flusherListMonitor;
    class FlusherEntry {
        friend class LogMgr;
        bool isOccupied;
        FlushFunction flusher;
        uint64_t recordPosToFlush;

        FlusherEntry() { isOccupied = false; }
    };
    std::vector<FlusherEntry> flushers;
    // the next pos is the position of the next log record that will be inserted
    // positions for currently hold records are in the interval
    // [nextRecordPos - records.size(), nextRecordPos - 1] (inclusive)
    uint64_t nextRecordPos;
    uint64_t oldestRecordPosToFlush;
    volatile LogLevel globalLogLevel;
    volatile bool isSerialImmediate; // print immediately to Serial, before putting in buffer

    void format(String *str, const char *format, int valueCount, LogValue **values);

friend class LogService;
    void callFlushers();
public:
    LogMgr();
    void init();
    ~LogMgr();

    Logger *newLogger(const char *name);
    void deleteLogger(Logger *logger);
    void doLog(const char *name, LogLevel level, const char *format, int valCount,
        const LogValue *val1 = nullptr, const LogValue *val2 = nullptr,
        const LogValue *val3 = nullptr, const LogValue *val4 = nullptr,
        const LogValue *val5 = nullptr, const LogValue *val6 = nullptr,
        const LogValue *val7 = nullptr, const LogValue *val8 = nullptr);
    void setGlobalLevel(LogLevel level);
    LogLevel getGlobalLevel();
    // clears the log before resizing buffer to new capacity
    void setCapacity(int records, int values);
    int getRecordsCapacity();
    void clear();
    void setSerialImmediate(bool enable);

    uint64_t getFirstRecordIdx();
    uint64_t getLastRecordIdx();
    bool getRecord(uint64_t idx, String *name, uint32_t *timestamp, LogLevel *level, String *str);
    const char *levelName(LogLevel level);

    FlusherHandle addFlusher(FlushFunction flushFunction);
    void removeFlusher(FlusherHandle handle);
};

class Logger {
friend class LogMgr;
    LogMgr *logMgr;
    const char *name;
    LogLevel logLevel;

    Logger(LogMgr *logMgr, const char *name, LogLevel logLevel);
private:
    ~Logger(); // to delete a logger call LogMgr::deleteLogger()
public:
    void setLevel(LogLevel level);
    LogLevel getLevel();
    bool isTrace();
    bool isDebug();
    bool isInfo();
    bool isWarn();
    bool isError();
    bool isFatal();

    void log(LogLevel level, const char *format);
    void log(LogLevel level, const char *format, const LogValue &val1);
    void log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2);
    void log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3);
    void log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4);
    void log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5);
    void log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5, const LogValue &val6);
    void log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5, const LogValue &val6, const LogValue &val7);
    void log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5, const LogValue &val6, const LogValue &val7, const LogValue &val8);

    void trace(const char *format);
    void trace(const char *format, const LogValue &val1);
    void trace(const char *format, const LogValue &val1, const LogValue &val2);
    void trace(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3);
    void trace(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4);
    void trace(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5);
    void trace(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5, const LogValue &val6);

    void debug(const char *format);
    void debug(const char *format, const LogValue &val1);
    void debug(const char *format, const LogValue &val1, const LogValue &val2);
    void debug(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3);
    void debug(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4);
    void debug(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5);
    void debug(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5, const LogValue &val6);
    void debug(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5, const LogValue &val6, const LogValue &val7);

    void info(const char *format);
    void info(const char *format, const LogValue &val1);
    void info(const char *format, const LogValue &val1, const LogValue &val2);
    void info(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3);
    void info(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4);
    void info(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5);
    void info(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5, const LogValue &val6);

    void warn(const char *format);
    void warn(const char *format, const LogValue &val1);
    void warn(const char *format, const LogValue &val1, const LogValue &val2);
    void warn(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3);
    void warn(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4);
    void warn(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5);
    void warn(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5, const LogValue &val6);

    void error(const char *format);
    void error(const char *format, const LogValue &val1);
    void error(const char *format, const LogValue &val1, const LogValue &val2);
    void error(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3);
    void error(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4);
    void error(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5);
    void error(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5, const LogValue &val6);

    void fatal(const char *format);
    void fatal(const char *format, const LogValue &val1);
    void fatal(const char *format, const LogValue &val1, const LogValue &val2);
    void fatal(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3);
    void fatal(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4);
    void fatal(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5);
    void fatal(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
        const LogValue &val5, const LogValue &val6);

};

class LogService {
private:
    LogMgr *logMgr;
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    UEventLoopTimer testTimer;
    UEventLoopTimer logFlusherTimer;
    Logger *logger;

#ifdef LOGGING_ENABLE_TESTS
    bool isTesting;
    const char *currentTestName;
    void checkRecord(uint64_t idx, const char *msgToCompare);
    void testStart(const char *testName);
    void testEnd();
#endif

    void getLast(int val, String *msg, int minLevel);

#ifdef LOGGING_USE_SYSLOG
private:
    String syslogServer;
    int syslogPort;
#endif

public:
    LogService();
    void init(LogMgr *logMgr, UEventLoop *eventLoop, CommandMgr *commandMgr);

};



//
// Inline methods
//

inline LogValue::LogValue()
{
    type = Type::VOID;
    format = nullptr;
}

inline LogValue::~LogValue()
{
    if (type == Type::STR) {
        if (this->val.cstrVal != nullptr) {
            free((void *)this->val.cstrVal);
        }
    } else if (type == Type::FN) {
        val.fnVal = nullptr;
    }
}

inline LogValue::LogValue(int val, const char *format)
{
    type = Type::INT;
    this->val.int64Val = val;
    this->format = format;
}

inline LogValue::LogValue(unsigned int val, const char *format)
{
    type = Type::UINT;
    this->val.int64Val = val;
    this->format = format;
}

inline LogValue::LogValue(int64_t val, const char *format)
{
    type = Type::INT;
    this->val.int64Val = val;
    this->format = format;
}

inline LogValue::LogValue(uint64_t val, const char *format)
{
    type = Type::UINT;
    this->val.int64Val = val;
    this->format = format;
}
inline LogValue::LogValue(double val, const char *format)
{
    type = Type::DOUBLE;
    this->val.doubleVal = val;
    this->format = format;
}
inline LogValue::LogValue(const char *val, const char *format)
{
    type = Type::STR;
    this->val.cstrVal = strdup(val);
    this->format = format;
}

inline LogValue::LogValue(std::function<void (String *)> val, const char *format)
{
    this->val.fnVal = val;
    type = Type::FN;
    this->format = format;
}

// moves ownership of strings-to-be-freed to this
inline LogValue::LogValue(const LogValue &other)
{
    type = other.type;
    switch (other.type) {
        case Type::BOOL: val.boolVal = other.val.boolVal; break;
        case Type::INT:
        case Type::UINT: val.int64Val = other.val.int64Val; break;
        case Type::DOUBLE: val.doubleVal = other.val.doubleVal; break;
        case Type::FLOAT: val.floatVal = other.val.floatVal; break;
        case Type::STR: val.cstrVal = other.val.cstrVal; const_cast<LogValue&>(other).val.cstrVal = nullptr; break;
        case Type::FN: val.fnVal = other.val.fnVal; const_cast<LogValue&>(other).val.fnVal = nullptr; break;
        case Type::VOID: break;
    }
    format = other.format;
}

// moves ownership of strings-to-be-freed to this
inline void LogValue::set(const LogValue &other)
{
    if (type == STR && val.cstrVal != nullptr) {
        free((void*)val.cstrVal);
    }
    type = other.type;
    switch (other.type) {
        case Type::BOOL: val.boolVal = other.val.boolVal; break;
        case Type::INT:
        case Type::UINT: val.int64Val = other.val.int64Val; break;
        case Type::DOUBLE: val.doubleVal = other.val.doubleVal; break;
        case Type::FLOAT: val.floatVal = other.val.floatVal; break;
        case Type::STR: val.cstrVal = other.val.cstrVal; const_cast<LogValue&>(other).val.cstrVal = nullptr; break;
        case Type::FN: val.fnVal = other.val.fnVal; const_cast<LogValue&>(other).val.fnVal = nullptr; break;
        case Type::VOID: break;
    }
    format = other.format;
}

inline void LogValue::clear()
{
    if (type == Type::STR) {
        if (this->val.cstrVal != nullptr) {
            free((void *)this->val.cstrVal);
        }
    } else if (type == Type::FN) {
        val.fnVal = nullptr;
    }
    type = Type::VOID;
}

inline LogRecord::LogRecord()
{
    name = nullptr;
    level = LogLevel::LOGLVL_FATAL;
    format = nullptr;
    startingValueIdx = 0;
    valueCount = 0;
}

inline void LogRecord::set(const char *name, LogLevel level, const char *format, int startingValueIdx, int valueCount)
{
    this->name = name;
    this->timestamp = esp_log_timestamp();
    this->level = level;
    this->format = format;
    this->startingValueIdx = startingValueIdx;
    this->valueCount = valueCount;
}

inline void LogRecord::set(const LogRecord &other)
{
    this->name = other.name;
    this->timestamp = other.timestamp;
    this->level = other.level;
    this->format = other.format;
    this->startingValueIdx = other.startingValueIdx;
    this->valueCount = other.valueCount;
}

inline void LogRecord::clear()
{
    this->name = nullptr;
    this->timestamp = 0;
    this->level = LogLevel::LOGLVL_FATAL;
    this->format = nullptr;
    this->startingValueIdx = -1;
    this->valueCount = 0;
}

inline Logger::Logger(LogMgr *logMgr, const char *name, LogLevel logLevel)
: logMgr(logMgr), name(name), logLevel(logLevel)
{
}

inline Logger::~Logger()
{
    logMgr = nullptr;
    name = nullptr;
}

inline void Logger::setLevel(LogLevel level)
{
    logLevel = level;
}

inline LogLevel Logger::getLevel()
{
    return logLevel;
}

inline bool Logger::isTrace()
{
    return logLevel >= LogLevel::LOGLVL_TRACE;
}

inline bool Logger::isDebug()
{
    return logLevel >= LogLevel::LOGLVL_DEBUG;
}

inline bool Logger::isInfo()
{
    return logLevel >= LogLevel::LOGLVL_INFO;
}

inline bool Logger::isWarn()
{
    return logLevel >= LogLevel::LOGLVL_WARN;
}

inline bool Logger::isError()
{
    return logLevel >= LogLevel::LOGLVL_ERROR;
}

inline bool Logger::isFatal()
{
    return logLevel >= LogLevel::LOGLVL_FATAL;
}

inline void Logger::log(LogLevel level, const char *format)
{
    if (level <= logLevel) {
        logMgr->doLog(name, level, format, 0);
    }
}
inline void Logger::log(LogLevel level, const char *format, const LogValue &val1)
{
    if (level <= logLevel) {
        logMgr->doLog(name, level, format, 1, &val1);
    }
}
inline void Logger::log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2)
{
    if (level <= logLevel) {
        logMgr->doLog(name, level, format, 2, &val1, &val2);
    }
}
inline void Logger::log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3)
{
    if (level <= logLevel) {
        logMgr->doLog(name, level, format, 3, &val1, &val2, &val3);
    }
}
inline void Logger::log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4)
{
    if (level <= logLevel) {
        logMgr->doLog(name, level, format, 4, &val1, &val2, &val3, &val4);
    }
}
inline void Logger::log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
    const LogValue &val5)
{
    if (level <= logLevel) {
        logMgr->doLog(name, level, format, 5, &val1, &val2, &val3, &val4, &val5);
    }
}
inline void Logger::log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
    const LogValue &val5, const LogValue &val6)
{
    if (level <= logLevel) {
        logMgr->doLog(name, level, format, 6, &val1, &val2, &val3, &val4, &val5, &val6);
    }
}
inline void Logger::log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
    const LogValue &val5, const LogValue &val6, const LogValue &val7)
{
    if (level <= logLevel) {
        logMgr->doLog(name, level, format, 7, &val1, &val2, &val3, &val4, &val5, &val6, &val7);
    }
}
inline void Logger::log(LogLevel level, const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4,
    const LogValue &val5, const LogValue &val6, const LogValue &val7, const LogValue &val8)
{
    if (level <= logLevel) {
        logMgr->doLog(name, level, format, 8, &val1, &val2, &val3, &val4, &val5, &val6, &val7, &val8);
    }
}

inline void Logger::trace(const char *format)
{
    logMgr->doLog(name, LogLevel::LOGLVL_TRACE, format, 0);
}
inline void Logger::trace(const char *format, const LogValue &val1)
{
    logMgr->doLog(name, LogLevel::LOGLVL_TRACE, format, 1, &val1);
}
inline void Logger::trace(const char *format, const LogValue &val1, const LogValue &val2)
{
    logMgr->doLog(name, LogLevel::LOGLVL_TRACE, format, 2, &val1, &val2);
}
inline void Logger::trace(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3)
{
    logMgr->doLog(name, LogLevel::LOGLVL_TRACE, format, 3, &val1, &val2, &val3);
}
inline void Logger::trace(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4)
{
    logMgr->doLog(name, LogLevel::LOGLVL_TRACE, format, 4, &val1, &val2, &val3, &val4);
}
inline void Logger::trace(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4, const LogValue &val5)
{
    logMgr->doLog(name, LogLevel::LOGLVL_TRACE, format, 5, &val1, &val2, &val3, &val4, &val5);
}
inline void Logger::trace(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4, const LogValue &val5, const LogValue &val6)
{
    logMgr->doLog(name, LogLevel::LOGLVL_TRACE, format, 6, &val1, &val2, &val3, &val4, &val5, &val6);
}


inline void Logger::debug(const char *format)
{
    logMgr->doLog(name, LogLevel::LOGLVL_DEBUG, format, 0);
}
inline void Logger::debug(const char *format, const LogValue &val1)
{
    logMgr->doLog(name, LogLevel::LOGLVL_DEBUG, format, 1, &val1);
}
inline void Logger::debug(const char *format, const LogValue &val1, const LogValue &val2)
{
    logMgr->doLog(name, LogLevel::LOGLVL_DEBUG, format, 2, &val1, &val2);
}
inline void Logger::debug(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3)
{
    logMgr->doLog(name, LogLevel::LOGLVL_DEBUG, format, 3, &val1, &val2, &val3);
}
inline void Logger::debug(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4)
{
    logMgr->doLog(name, LogLevel::LOGLVL_DEBUG, format, 4, &val1, &val2, &val3, &val4);
}
inline void Logger::debug(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4, const LogValue &val5)
{
    logMgr->doLog(name, LogLevel::LOGLVL_DEBUG, format, 5, &val1, &val2, &val3, &val4, &val5);
}
inline void Logger::debug(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4, const LogValue &val5, const LogValue &val6)
{
    logMgr->doLog(name, LogLevel::LOGLVL_DEBUG, format, 6, &val1, &val2, &val3, &val4, &val5, &val6);
}
inline void Logger::debug(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4, const LogValue &val5, const LogValue &val6, const LogValue &val7)
{
    logMgr->doLog(name, LogLevel::LOGLVL_DEBUG, format, 6, &val1, &val2, &val3, &val4, &val5, &val6, &val7);
}



inline void Logger::info(const char *format)
{
    logMgr->doLog(name, LogLevel::LOGLVL_INFO, format, 0);
}
inline void Logger::info(const char *format, const LogValue &val1)
{
    logMgr->doLog(name, LogLevel::LOGLVL_INFO, format, 1, &val1);
}
inline void Logger::info(const char *format, const LogValue &val1, const LogValue &val2)
{
    logMgr->doLog(name, LogLevel::LOGLVL_INFO, format, 2, &val1, &val2);
}
inline void Logger::info(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3)
{
    logMgr->doLog(name, LogLevel::LOGLVL_INFO, format, 3, &val1, &val2, &val3);
}
inline void Logger::info(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4)
{
    logMgr->doLog(name, LogLevel::LOGLVL_INFO, format, 4, &val1, &val2, &val3, &val4);
}
inline void Logger::info(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4, const LogValue &val5)
{
    logMgr->doLog(name, LogLevel::LOGLVL_INFO, format, 5, &val1, &val2, &val3, &val4, &val5);
}
inline void Logger::info(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4, const LogValue &val5, const LogValue &val6)
{
    logMgr->doLog(name, LogLevel::LOGLVL_INFO, format, 6, &val1, &val2, &val3, &val4, &val5, &val6);
}


inline void Logger::warn(const char *format)
{
    logMgr->doLog(name, LogLevel::LOGLVL_WARN, format, 0);
}
inline void Logger::warn(const char *format, const LogValue &val1)
{
    logMgr->doLog(name, LogLevel::LOGLVL_WARN, format, 1, &val1);
}
inline void Logger::warn(const char *format, const LogValue &val1, const LogValue &val2)
{
    logMgr->doLog(name, LogLevel::LOGLVL_WARN, format, 2, &val1, &val2);
}
inline void Logger::warn(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3)
{
    logMgr->doLog(name, LogLevel::LOGLVL_WARN, format, 3, &val1, &val2, &val3);
}
inline void Logger::warn(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4)
{
    logMgr->doLog(name, LogLevel::LOGLVL_WARN, format, 4, &val1, &val2, &val3, &val4);
}
inline void Logger::warn(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4, const LogValue &val5)
{
    logMgr->doLog(name, LogLevel::LOGLVL_WARN, format, 5, &val1, &val2, &val3, &val4, &val5);
}
inline void Logger::warn(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4, const LogValue &val5, const LogValue &val6)
{
    logMgr->doLog(name, LogLevel::LOGLVL_WARN, format, 6, &val1, &val2, &val3, &val4, &val5, &val6);
}


inline void Logger::error(const char *format)
{
    logMgr->doLog(name, LogLevel::LOGLVL_ERROR, format, 0);
}
inline void Logger::error(const char *format, const LogValue &val1)
{
    logMgr->doLog(name, LogLevel::LOGLVL_ERROR, format, 1, &val1);
}
inline void Logger::error(const char *format, const LogValue &val1, const LogValue &val2)
{
    logMgr->doLog(name, LogLevel::LOGLVL_ERROR, format, 2, &val1, &val2);
}
inline void Logger::error(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3)
{
    logMgr->doLog(name, LogLevel::LOGLVL_ERROR, format, 3, &val1, &val2, &val3);
}
inline void Logger::error(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4)
{
    logMgr->doLog(name, LogLevel::LOGLVL_ERROR, format, 4, &val1, &val2, &val3, &val4);
}
inline void Logger::error(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4, const LogValue &val5)
{
    logMgr->doLog(name, LogLevel::LOGLVL_ERROR, format, 5, &val1, &val2, &val3, &val4, &val5);
}
inline void Logger::error(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4, const LogValue &val5, const LogValue &val6)
{
    logMgr->doLog(name, LogLevel::LOGLVL_ERROR, format, 6, &val1, &val2, &val3, &val4, &val5, &val6);
}


inline void Logger::fatal(const char *format)
{
    logMgr->doLog(name, LogLevel::LOGLVL_FATAL, format, 0);
}
inline void Logger::fatal(const char *format, const LogValue &val1)
{
    logMgr->doLog(name, LogLevel::LOGLVL_FATAL, format, 1, &val1);
}
inline void Logger::fatal(const char *format, const LogValue &val1, const LogValue &val2)
{
    logMgr->doLog(name, LogLevel::LOGLVL_FATAL, format, 2, &val1, &val2);
}
inline void Logger::fatal(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3)
{
    logMgr->doLog(name, LogLevel::LOGLVL_FATAL, format, 3, &val1, &val2, &val3);
}
inline void Logger::fatal(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4)
{
    logMgr->doLog(name, LogLevel::LOGLVL_FATAL, format, 4, &val1, &val2, &val3, &val4);
}
inline void Logger::fatal(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4, const LogValue &val5)
{
    logMgr->doLog(name, LogLevel::LOGLVL_FATAL, format, 5, &val1, &val2, &val3, &val4, &val5);
}
inline void Logger::fatal(const char *format, const LogValue &val1, const LogValue &val2, const LogValue &val3, const LogValue &val4, const LogValue &val5, const LogValue &val6)
{
    logMgr->doLog(name, LogLevel::LOGLVL_FATAL, format, 6, &val1, &val2, &val3, &val4, &val5, &val6);
}

/////////////////////////////
//
// Queue inline methods
//
/////////////////////////////

template < class T >
inline Queue<T>::Queue()
{
    length = 256;
    buf = new T[length];
    headPosition = 0;
    _nextIdx = 0;
    _tailIdx = 0;
}

template < class T >
inline Queue<T>::~Queue()
{
    delete[] buf;
}

template < class T >
inline int Queue<T>::capacity()
{
    return length - 1; // one less, as _headIdx == _tailIdx means queue empty
}

template < class T >
inline void Queue<T>::capacity(int n)
{
    int s = size();
    int newLength = n + 1;
    if (newLength < s + 1) {
        newLength = s + 1;
    }
    T *newBuf = new T[newLength];
    if (_nextIdx >= _tailIdx) {
        memcpy(&newBuf[0], &buf[_tailIdx], s);
    } else {
        memcpy(&newBuf[0], &buf[_tailIdx], length - _tailIdx);
        memcpy(&newBuf[length - _tailIdx], &buf[_nextIdx], _nextIdx);
    }
    delete[] buf;
    buf = newBuf;
    length = newLength;
    _tailIdx = 0;
    _nextIdx = s;
}

template < class T >
inline int Queue<T>::size()
{
    if (_nextIdx >= _tailIdx) {
        return _nextIdx - _tailIdx;
    } else {
        return _nextIdx - _tailIdx + length;
    }
}

template < class T >
inline int Queue<T>::headIdx()
{
    return _nextIdx == 0 ? length - 1 : _nextIdx - 1;
}

template < class T >
inline int Queue<T>::nextIdx()
{
    return _nextIdx;
}

template < class T >
inline int Queue<T>::tailIdx()
{
    return _tailIdx;
}

template < class T >
inline bool Queue<T>::canAddHead()
{
    return canAddHead(1);
}

template < class T >
inline bool Queue<T>::canAddHead(int n)
{
    int s = size();
    return s + n < length - 1;
}

template < class T >
inline T *Queue<T>::addHead()
{
    T *p = &buf[_nextIdx];
    ++_nextIdx;
    if (_nextIdx == length) {
        _nextIdx = 0;
    }
    return p;
}

template < class T >
inline void Queue<T>::addHead(int n)
{
    _nextIdx += n;
    if (_nextIdx >= length) {
        _nextIdx -= length;
    }
}

// returns null if idx is not in range [_tailIdx .. _nextIdx - 1] (% length)
template < class T >
inline T *Queue<T>::at(int idx)
{
    if (idx < 0) {
        idx += length;
    }
    if (idx >= length) {
        idx -= length;
    }
    if (_nextIdx >= _tailIdx) {
        return (idx >= _tailIdx && idx < _nextIdx) ? &buf[idx] : nullptr;
    } else {
        return (idx >= _nextIdx && idx < _tailIdx) ? nullptr : &buf[idx];
    }
}

template < class T >
inline T *Queue<T>::atHead()
{
    if (_nextIdx == _tailIdx) {
        return nullptr;
    } else if (_nextIdx == 0) {
        return &buf[length - 1];
    } else {
        return &buf[_nextIdx - 1];
    }
}

template < class T >
inline T *Queue<T>::atHead(int i)
{
    if (i >= size()) {
        return nullptr;
    }
    int p = _nextIdx - 1 - i;
    if (p < 0) {
        p += length;
    }
    return &buf[p];
}

template < class T >
inline T *Queue<T>::atTail()
{
    if (_nextIdx == _tailIdx) {
        return nullptr;
    }
    return &buf[_tailIdx];
}

template < class T >
inline T *Queue<T>::atTail(int i)
{
    if (i >= size()) {
        return nullptr;
    }
    int p = _tailIdx + i >= length ? _tailIdx + i - length : _tailIdx + i;
    return &buf[p];
}

template < class T >
inline bool Queue<T>::canRemoveTail()
{
    return _nextIdx != _tailIdx;
}

template < class T >
inline bool Queue<T>::canRemoveTail(int n)
{
    return size() >= n;
}

template < class T >
inline void Queue<T>::removeTail()
{
    ++_tailIdx;
    if (_tailIdx == length) {
        _tailIdx = 0;
    }
}

template < class T >
inline void Queue<T>::removeTail(int n)
{
    _tailIdx += n;
    if (_tailIdx >= length) {
        _tailIdx -= length;
    }
}

template < class T >
inline void Queue<T>::clear()
{
    _tailIdx = 0;
    _nextIdx = 0;
}

#endif

#endif // LOGMGR_H

