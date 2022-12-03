#include <CompilationOpts.h>

#ifdef USE_PCF8563

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "RtcPcf8563.h"
#include "LogMgr.h"
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <Wire.h>
#include "pcf8563.h"

// for some reason VC doesn't detect the declaration of setenv, though compilation is ok
extern int setenv(const char *name, const char *value, int overwrite);

void RtcPcf8563Service::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr, SystemService *systemService)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->systemService = systemService;
    this->logger = logMgr->newLogger("rtc");

    ServiceCommands *cmd = commandMgr->getServiceCommands("rtc");
    initCommands(cmd);

    // defaults
    isEnabled = false;
    i2cPort = 0;
    i2cAddr = 0x51;
    isAutoSyncSystemTime = true;
    tzInfo = "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00";

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

    // init hardware
    if (!isEnabled) {
        logger->info("Service is not enabled");
    } else {
        rc = systemService->getIdf(i2cPort);
        if (!rc) {
            logger->error("Couldn't get i2c driver for port {} using IDF, RTC will not work", i2cPort);
        } else {
            rtc.begin((i2c_port_t)i2cPort, i2cAddr);

            // get date from RTC and use it to set the system date
            if (isAutoSyncSystemTime) {
                setSystemTime();
            }
        }
    }
}

void RtcPcf8563Service::initCommands(ServiceCommands *cmd)
{
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("enable", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .helpOn("--> Enable the service (needs reboot if it was not enabled during startup)")
        .helpOff("--> Disable the service")
        .ptr(&isEnabled)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("i2cPort", true)
        .cmd("i2cPort")
        .help("--> Set the i2c port that will be used (0 or 1); i2c pins are set in system service. Requires save and reboot")
        .vMin(0)
        .vMax(1)
        .ptr(&i2cPort)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("i2cAddr", true)
        .cmd("i2cAddr")
        .help("--> Set i2c address, requires save and reboot")
        .vMin(-1)
        .vMax(99)
        .ptr(&i2cAddr)
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("clock", true)
        .cmd("clock")
        .help("--> Enable CLK to a given frequency in Hz (1, 32, 1024, 32768), or \"off\" to disable it")
        .isPersistent(false)
        .setFn([this](const String &val, bool isLoading, String *msg) {
            if (val == "1") {
                rtc.enableCLK(PCF8563_CLK_1HZ);
                *msg = "Clock set to 1 Hz";
            } else if (val == "32") {
                rtc.enableCLK(PCF8563_CLK_32HZ);
                *msg = "Clock set to 32 Hz";
            } else if (val == "1024") {
                rtc.enableCLK(PCF8563_CLK_1024KHZ);
                *msg = "Clock set to 1024 Hz";
            } else if (val == "32768") {
                rtc.enableCLK(PCF8563_CLK_32_768KHZ);
                *msg = "Clock set to 32768 Hz";
            } else if (val == "off" || val == "OFF") {
                rtc.disableCLK();
                *msg = "Clock disabled";
            } else {
                *msg = "Expecting 1, 32, 1024, 32768 or off, instead of \"" + val + "\"";
            }
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("timer", true)
        .cmd("timer")
        .help("--> timer off | clear | <n>: Set timer in milliseconds. The <n> can be suffixed with \"s\", \"m\" or \"h\"\n"
                "        to mean respectively seconds, minutes or hours.\n"
                "        Timer period is approximative; first a frequency is chosen from 1/60 Hz, 1 Hz, 64 Hz, 4096 Hz,\n"
                "        then a 8-bit counter is set.\n"
                "        The command \"timer clear\" will clear the interrupt status of the timer.")
        .isPersistent(false)
        .setFn([this](const String &val, bool isLoading, String *msg) {
            if (val.equalsIgnoreCase("off")) {
                timerDisable();
                *msg = "Timer disabled";
            } else {
                uint32_t count;
                int last = 0; // trim end of string (beginning is trimmed by cmd)
                while (last < val.length() && val.charAt(last) >= '0' && val.charAt(last) <= '9') {
                    ++last;
                }
                if (last == val.length()) {
                    count = val.toInt();
                } else {
                    char lastChar = val.charAt(last);
                    count = val.toInt();
                    switch (lastChar) {
                        case 'h': count *= 1000 * 60 * 60; break;
                        case 'm': count *= 1000 * 60; break;
                        case 's': count *= 1000; break;
                        default: {
                            *msg = "Unrecognized number suffix: \"%c\", only \"h\", \"m\" or \"s\" expected";
                            count = 0;
                        }
                    }
                }
                if (count > 0) {
                    uint32_t duration = timerEnable(count, TIMER_ROUND_DOWN);
                    uint32_t h, m, s, ms;
                    h = duration / 3600000;
                    m = (duration - h * 3600000) / 60000;
                    s = (duration - h * 3600000 - m * 60000) / 1000;
                    ms = (duration - h * 3600000 - m * 60000 - s * 1000);
                    *msg = "Timer set at ";
                    *msg += duration;
                    *msg += String(" millis, ") + h + "h " + m + "m " + s + "s " + ms + "ms";
                }
            }
            return true;
        })
        .getFn([this](String *val) {
            bool isEnabled = rtc.isTimerEnable();
            bool isActive = rtc.isTimerActive();
            *val = String("Timer is ") + (isEnabled ? "enabled" : "disabled") + ", " + (isActive ? "active" : "inactive");
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("autoSyncSystemTime", true)
        .cmd("autoSyncSystemTime")
        .help("--> If set, system time is set to RTC time at initialization and after \"setTime\" command")
        .ptr(&isAutoSyncSystemTime)
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("timeZoneInfo", true)
        .cmd("timeZoneInfo")
        .help("--> Timezone info, e.g.: CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00, see https://remotemonitoringsystems.ca/time-zone-abbreviations.php")
        .ptr(&tzInfo)
        .setFn([this](const String &val, bool isLoading, String *msg) {
            tzInfo = val;
            setenv("TZ", tzInfo.c_str(), 1);
            *msg = "TZ set to "; msg->concat(tzInfo);
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("showTime", true)
        .cmdOn("showTime")
        .helpOn("--> Show current RTC and system time")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            *msg = "RTC time UTC: ";
            getRtcTime(msg);
            *msg += "\nRTC time at local timezone: ";
            getRtcTimeLocal(msg);
            msg->concat("\nCurrent system time: ");
            getSystemTime(msg);
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("time", true)
        .cmd("time")
        .help("--> Set RTC time, format \"YYYY-MM-DD HH:MI:SS\" at current time zone and, if autoSyncSystemTime is set, system time is also set")
        .isPersistent(false)
        .getFn([this](String *val) {
            getRtcTimeLocal(val);
        })
        .setFn([this](const String &val, bool isLoading, String *msg) {
            struct tm tm = { 0 };
            int rc = sscanf(val.c_str(), "%04d-%02d-%02d %02d:%02d:%02d",
                &tm.tm_year, &tm.tm_mon, &tm.tm_mday, &tm.tm_hour, &tm.tm_min, &tm.tm_sec);
            if (rc != 6) {
                *msg = "Incorrect input format";
                return true;
            }

            tm.tm_isdst = -1; // take in account TZ
            tm.tm_year -= 1900;
            tm.tm_mon -= 1;

            setenv("TZ", tzInfo.c_str(), 1);

            time_t unixTime = mktime(&tm);
            if (unixTime == -1) {
                *msg = "Error converting input to unix time";
                return true;
            }
            struct tm utcTm;
            gmtime_r(&unixTime, &utcTm);

            rtc.setDateTime(utcTm.tm_year + 1900, utcTm.tm_mon + 1, utcTm.tm_mday, utcTm.tm_hour, utcTm.tm_min, utcTm.tm_sec);

            if (isAutoSyncSystemTime) {
                setSystemTime();
            }

            *msg = "RTC time UTC: ";
            getRtcTime(msg);
            *msg += "\nRTC time at local timezone: ";
            getRtcTimeLocal(msg);
            msg->concat("\nCurrent system time: ");
            getSystemTime(msg);
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("initSystemTime", true)
        .cmdOn("initSystemTime")
        .helpOn("--> Set system time from the RTC time (considered at UTC)")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            setSystemTime();
            *msg = "Current system time: ";
            getSystemTime(msg);
            return true;
        })
    );

}

void RtcPcf8563Service::clockDisable() {
    if (!isEnabled) {
        return;
    }
    rtc.disableCLK();
}

bool RtcPcf8563Service::clockEnable(int freq) {
    if (!isEnabled) {
        return false;
    }
    switch (freq) {
        case 1: rtc.enableCLK(PCF8563_CLK_1HZ); return true;
        case 32: rtc.enableCLK(PCF8563_CLK_32HZ); return true;
        case 1024: rtc.enableCLK(PCF8563_CLK_1024KHZ); return true;
        case 32768: rtc.enableCLK(PCF8563_CLK_32_768KHZ); return true;
        default: return false;
    }
}

void RtcPcf8563Service::timerDisable() {
    if (!isEnabled) {
        return;
    }
    rtc.disableTimer();
    rtc.clearTimer();
}

uint32_t RtcPcf8563Service::timerEnable(int millis, TIMER_ROUND_DIRECTION dir)
{
    if (!isEnabled) {
        return millis;
    }
    uint8_t freq = 0;
    uint8_t timerCounter = 0;
    uint32_t actualDuration = 0;
    bool dirUp = (dir == TIMER_ROUND_UP);

    // find which frequency we can use, out of 1/60, 1, 64, 4096 Hz
    if (millis < 256 * 1000 / 4096) {
        freq = 0b00; // 4096 Hz
        timerCounter = (millis * 4096 + (dirUp ? 999 : 0)) / 1000;
        actualDuration = timerCounter * 1000 / 4096;
    } else if (millis <= 255 * 1000 / 64) {
        freq = 0b01; // 64 Hz
        timerCounter = (millis * 64 + (dirUp ? 999 : 0)) / 1000;
        actualDuration = timerCounter * 1000 / 64;
    } else if (millis <= 255 * 1000) {
        freq = 0b10; // 1 Hz
        timerCounter = (millis * 1 + (dirUp ? 999 : 0)) / 1000;
        actualDuration = timerCounter * 1000 / 1;
    } else if (millis <= 255 * 1000 * 60) {
        freq = 0b11; // 1/60 Hz
        timerCounter = (millis + (dirUp ? (1000 * 60 - 1) : 0)) / 1000 / 60;
        actualDuration = timerCounter * 1000 * 60;
    } else { // too big a duration, set to maximum
        freq = 0b11; // 1/60 Hz
        timerCounter = 255u;
        actualDuration = timerCounter * 1000 * 60;
    }

    rtc.clearTimer();
    rtc.setTimer(timerCounter, freq, true);
    rtc.enableTimer();
    return actualDuration;
}

void RtcPcf8563Service::setSystemTime()
{
    if (!isEnabled) {
        return;
    }
    // https://github.com/G6EJD/ESP32-Time-Services-and-SETENV-variable/blob/master/ESP32_Time_and_SETENV.ino

    // See https://github.com/nayarsystems/posix_tz_db/blob/master/zones.csv for Timezone codes for your region
    // See: https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
    // CST6CDT, M3.2.0/2:00:00, M11.1.0/2:00:00
    // CST6CDT is the name of the time zone
    // CST is the abbreviation used when DST is off
    // 6 hours is the time difference from GMT
    // CDT is the abbreviation used when DST is on
    // ,M3 is the third month
    // .2 is the second occurrence of the day in the month
    // .0 is Sunday
    // /2:00:00 is the time
    // ,M11 is the eleventh month
    // .1 is the first occurrence of the day in the month
    // .0 is Sunday
    // 2:00:00 is the time
    // enter your time zone (https://remotemonitoringsystems.ca/time-zone-abbreviations.php)
    setenv("TZ", tzInfo.c_str(), 1);
    struct timeval tv;
    tv.tv_sec = rtcTime();
    tv.tv_usec = 0;
    settimeofday(&tv, NULL);
}

void RtcPcf8563Service::getSystemTime(String *msg)
{
    time_t now;
    time(&now);
    struct tm timeInfo;
    localtime_r(&now, &timeInfo);
    char timeOutput[30];
    strftime(timeOutput, sizeof(timeOutput), "%F %T%z", &timeInfo);
    msg->concat(timeOutput);
}

void RtcPcf8563Service::getRtcTime(String *msg)
{
    if (!isEnabled) {
        *msg = "1970-01-01 00:00:00";
        return;
    }
    RTC_Date dt = rtc.getDateTime();
    char buf[50];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
    msg->concat(buf);
}

time_t RtcPcf8563Service::rtcTime()
{
    if (!isEnabled) {
        return 0;
    }
    RTC_Date dt = rtc.getDateTime();

    static const int daysOfPrecedingMonths[] = { // per month, days of preceding monthss of the year
        0,
        31,
        31 + 28,
        31 + 28 + 31,
        31 + 28 + 31 + 30,
        31 + 28 + 31 + 30 + 31,
        31 + 28 + 31 + 30 + 31 + 30,
        31 + 28 + 31 + 30 + 31 + 30 + 31,
        31 + 28 + 31 + 30 + 31 + 30 + 31 + 31,
        31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30,
        31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31,
        31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 30
    };
    int yday = daysOfPrecedingMonths[dt.month - 1] + (dt.day - 1)
        + ((dt.year % 4) == 0 && (dt.year % 400) != 0 ? 1 : 0);

    // assumes time_t is seconds since 1970-01-01

    time_t time = dt.second + dt.minute * 60 + dt.hour * 3600
        + yday * 86400LL +
        (dt.year - 1900 - 70) * 31536000LL + ((dt.year  - 1900 - 69) / 4) * 86400LL -
        ((dt.year - 1900 - 1) / 100) * 86400LL + ((dt.year - 1900 + 299) / 400) * 86400LL;

    return time;
}

void RtcPcf8563Service::getRtcTimeLocal(String *msg)
{
    if (!isEnabled) {
        *msg = "1970-01-01 00:00:00";
        return;
    }
    time_t time = rtcTime();
    struct tm timeInfo;
    localtime_r(&time, &timeInfo);

    char timeOutput[30];
    strftime(timeOutput, sizeof(timeOutput), "%F %T%z", &timeInfo);

    msg->concat(timeOutput);
}

#endif
