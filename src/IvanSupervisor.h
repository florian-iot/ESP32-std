#include <CompilationOpts.h>
#ifdef USE_IVAN_SUPERVISOR
#ifndef INCL_IVAN_SUPERVISOR_H
#define INCL_IVAN_SUPERVISOR_H

// debug.h defines info() and other defines as used in Logger
// define the following so that debug.h is not included
#define _COAP_DEBUG_H_

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <deque>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "RtcPcf8563.h"
#include "Sim7000.h"
#include "Dfa.h"
#include "EEPROMFsService.h"
#include "ButtonService.h"
#include "WifiServiceAsync.h"
#include "Ina3221Service.h"
#include "LM75A_TempService.h"
#include "SystemService.h"
#include "AM2320Service.h"
#include "Ble.h"
#include "Esp32I2c.h"
#include "RebootDetectorService.h"
#include "LedIndicatorService.h"

// Telemetry tags

#define TELEMETRY_START "TS"
// + involuntary or voluntary (optional)
#define TELEMETRY_DATA_INVOLUNTARY_SHUTDOWN "IS"
#define TELEMETRY_SHUTDOWN "TD"
// + sleep duration
#define TELEMETRY_DATA_SLEEP_DURATION "SL"
#define TELEMETRY_SHUTDOWN_MANUALLY "TM"
#define TELEMETRY_SHUTDOWN_NOT_PERFORMED "TX"
#define TELEMETRY_LOAD_BEGIN "LB"
#define TELEMETRY_LOAD_END "LE"
// + overcurrent, current value (optional)
#define TELEMETRY_LOAD_END_ABORTED_OVERCURRENT "LO"
// load cannot be turned on because of protection conditions
#define TELEMETRY_LOAD_BEGIN_FORBIDDEN "LF"

#define TELEMETRY_UPLOAD_SENT_CONFIRMED "UC"
#define TELEMETRY_UPLOAD_SENT_NOT_CONFIRMED "UN"
#define TELEMETRY_UPLOAD_NOT_SENT "UE"
#define TELEMETRY_UPLOAD_SENT_NOT_CONFIRMED_CLEARED "UF"
#define TELEMETRY_UPLOAD_NOT_SENT_CLEARED "UG"
// used with TELEMETRY_UPLOAD_xx when not sent or when cleared
#define TELEMETRY_UPLOAD_GSM_RSSI "UR"
#define TELEMETRY_UPLOAD_SIM7000_ERROR "UO"
// used with TELEMETERY_UPLOAD_xx when cleared
#define TELEMETRY_CLEARED_LINES "UL"
#define TELEMETRY_CLEARED_START_COUNT "US"
#define TELEMETRY_CLEARED_FIRST_TS "UB"
#define TELEMETRY_CLEARED_LAST_TS "UT"

#define TELEMETRY_CMD_RETRIEVE_FAILURE "CF"
#define TELEMETRY_CMD_PROCESSED_SUCCESSFULLY "CS"
// + text CT, result CR
#define TELEMETRY_DATA_CMD_TEXT "CT"
#define TELEMETRY_DATA_CMD_RESULT "CR"
#define TELEMETRY_CMD_PROCESSED_FAILURE "CE"
// + text CT
#define TELEMETRY_CMD_IGNORED "CI"
// + text CT

// the following are only used in server side, in case of error
#define TELEMETRY_UNKNOWN_TYPE "UK"
#define TELEMETRY_BAD_DATA "BD"

// the following are SMS messages generated by Hologram
#define TELEMETRY_SMS_DT "SM"
#define TELEMETRY_SMS_DT_DELIVERED "SD"

// telemetry from sensor alerts and conditions

#define TELEMETRY_LOG_OVERCURRENT "PC"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_PROT_VALUE "PV"
#define TELEMETRY_CROWBAR "PB"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_NO_CROWBAR "PA"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_LOW_OUTPUT_VOLTAGE "PO"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_NORMAL_OUTPUT_VOLTAGE "PN"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_OUTPUT_IS_POWERED "PP"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_OUTPUT_IS_NOT_POWERED "PW"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_LOG_UNDERVOLTAGE "PU"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_TURN_OFF_UNDERVOLTAGE "PG"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_SHUTDOWN_UNDERVOLTAGE "PH"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_TURN_OFF_OVERTEMP "PF"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_SHUTDOWN_OVERTEMP "PD"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_OVERTEMP "PT"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_NOT_OVERTEMP "PM"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_TEMP_ERROR "PE"
#define TELEMETRY_TEMP_NOT_ERROR "PR"
// + value TELEMETRY_PROT_VALUE
#define TELEMETRY_SENSOR_LOAD_PROFILE "PS"
// a string, the current and voltage profile for a load session
#define TELEMETRY_SENSOR_LOAD_PROFILE_VALUE "PL"

#define TELEMETRY_SENSOR_DATA "SA"
// + values
#define TELEMETRY_SENSOR_DATA_MAIN_VOLTAGE "SV"
#define TELEMETRY_SENSOR_DATA_MAIN_CURRENT "SC"
#define TELEMETRY_SENSOR_DATA_LOAD_VOLTAGE "SW"
#define TELEMETRY_SENSOR_DATA_LOAD_CURRENT "SU"
#define TELEMETRY_SENSOR_DATA_CASE_TEMP "ST"
#define TELEMETRY_SENSOR_DATA_ENV_TEMP "SE"
#define TELEMETRY_SENSOR_DATA_ENV_HUMIDITY "SH"
#define TELEMETRY_SENSOR_DATA_ENV_BATTERY_PCT "SB"
#define TELEMETRY_SENSOR_DATA_ENV_RSSI "SI"
#define TELEMETRY_SENSOR_DATA_GSM_RSSI "SR"


class IvanSupervisorService;

class Sensors {
public:
    Sensors();
    void init(IvanSupervisorService *sup, ServiceCommands *cmd);
    void start();

    void setLoadState(bool on);
    bool protIsDoNotLoad(); // == protIsOutputPowered || protIsCrowbar || protIsUndervoltage

    void loadOnOvercurrentSupervisionPrepare();
    // returns the sensed current in case of overcurrent, else 0
    int loadOnOvercurrentSupervision(int millis);

    void spotMeasure();
    void readSensors(int *mainVoltage, int *mainCurrent, int *loadVoltage, int *loadCurrent,
        int *caseTemp, int *envTemp, int *envHumidity, int *sensorBatteryPct, int *sensorRssi, int *rssi);
    void readLoadProfile(String *loadProfile);

private:
    IvanSupervisorService *sup;
    UEventLoopTimer timer;

    int highFrequencyMillis;
    int lowFrequencyMillis;
    float hystCurrentPct;
    float hystVoltagePct;

    bool protIsEnabled; // for all protections except crowbar
    bool protCrowbarIsEnabled; // for crowbar protection
    int protOffMaxCurrent;
    int protOffMaxVoltage;
    int protOnMaxCurrent;
    int protOnMinVoltage;
    int protMinVoltageToLoad;
    int protMinVoltageShutdown;
    int protMaxTempToLoad;
    int protMaxTempShutdown;
    String bleSensorAddress;

    bool isLoadOn;
    unsigned long loadOnTs;
    unsigned long loadOffTs;
    bool isTurnOffInitiated;
    bool isShutdownInitiated;

    int protIsOutputPowered;
    int protIsCrowbar;
    int protIsUndervoltage;
    bool protIsLowOutputVoltage;
    bool protIsOvertemp;
    bool protIsTempError;

    int loadVoltage; // mV
    int loadCurrent; // mA
    int mainVoltage; // mV
    int mainCurrent; // mA
    int caseTemp; // milli°C

    bool isFirstProfile;
    unsigned long lastProfileTs;
    unsigned long lastMeasureTs;
    int lastLoadCurrent;
    int lastLoadVoltage;
    uint32_t measurementCounter;

    // Current during load, in compressed format -- see protobuf varint
    std::vector<uint8_t> currentProfile;
    void varintAdd(std::vector<uint8_t> &vect, int32_t n, bool isSignedEncoding);
    int32_t varintDecode(std::vector<uint8_t> &vect, std::vector<uint8_t>::iterator &ptr, bool isSignedEncoding);

    void initCommands(ServiceCommands *cmd);

    void doSensorMeasurements(bool isHighFreq);
};

class IvanSupervisorLedIndicators {
public:
    enum LedIndicatorEvent {
        LI_ON,
        LI_LOAD_RUNNING,
        LI_LOAD_DONE,
        LI_SHUTTING_DOWN,
        LI_BUTTON_CLICK,
        LI_BUTTON_LONG_CLICK
    };
private:
    LedIndicatorService *ledIndicator;
    int onNoLoad;
    int onWithLoad;
    int onWifiOnNoLoad;
    int onWifiOnWithLoad;
    int loadRunning;
    int loadRunningWifiOn;
    int idle;
    int idleNoShutdown;
    int idleWifiOn;
    int idleWifiOnNoShutdown;
    int idleShuttingDown;
    int buttonClick;
    int buttonLongClick;

    bool isWithLoad;
    bool isWithWifi;
    bool isWithAutoShutdown;

    LedIndicatorEvent lastEvent;
public:
    void init(UEventLoop *eventLoop, ServiceCommands *cmd, LedIndicatorService *ledIndicator, Logger *logger);
    void event(LedIndicatorEvent event);
    void setWithLoad(bool enabled);
    void setWithWifi(bool enabled);
    void setWithAutoShutdown(bool enabled);
};

class IvanSupervisorService {
    friend class Sensors;

public:
    IvanSupervisorService();
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr,
        SystemService *systemService, RebootDetectorService *rebootDetectorService,
        ButtonService *buttonService, EEPROMFsService *eepromFsService, WifiAsyncService *wifi,
        RtcPcf8563Service *rtc, Sim7000Service *sim7000, Ina3221Service *ina3221, LM75A_TempService *tempService,
#ifdef USE_AM2320
        AM2320Service *am2320,
#endif
#ifdef USE_ESP32I2C
        Esp32I2cService *esp32i2c,
#endif
#ifdef USE_BLE
        BleService *ble,
#endif
#ifdef USE_LED_INDICATOR
        LedIndicatorService *ledIndicator,
#endif
        LogMgr *logMgr);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;

    UEventLoopTimer  rtcTimer;
    void initCommands(ServiceCommands *cmd);
    void initDfa();
    void initLoadFlow();

    uint32_t getDuration(const String &val, bool isSeconds, String *msg);

    Dfa dfa; // must be declared before the states and inputs, so that it gets initialized before

#define defState(dfa, s) Dfa::State s = dfa.nextState(#s)
#define defInput(dfa, s) Dfa::Input s = dfa.nextInput(#s)

    defState(dfa, STARTUP);
    defState(dfa, IDLE);
    defState(dfa, DO_SHUTDOWN);
    defState(dfa, DO_SHUTDOWN_WITH_WAIT);
    defState(dfa, DO_SHUTDOWN_AFTER_WAIT);
    defState(dfa, SHUTTING_DOWN_S1);
    defState(dfa, SHUTTING_DOWN_S2);

    defInput(dfa, STARTUP_INITIATE);
    defInput(dfa, REQUEST_SHUTDOWN);
    defInput(dfa, REQUEST_SHUTDOWN_WITH_WAIT);
    defInput(dfa, CANCEL_SHUTDOWN);

    defState(dfa, LOAD_SMS_START);
    defState(dfa, LOAD_SMS_S01);
    defState(dfa, LOAD_SMS_RETRY);
    defState(dfa, LOAD_SMS_S10);
    defState(dfa, LOAD_SMS_S20);
    defState(dfa, LOAD_SMS_TERMINATED);

    defInput(dfa, REQUEST_LOAD_SMS);
    defInput(dfa, LOAD_SMS_UNMARSHALLED);
    defInput(dfa, LOAD_SMS_DELETED);
    defInput(dfa, LOAD_SMS_LOAD_SUCCEEDED);
    defInput(dfa, LOAD_SMS_LOAD_FAILED);

    defState(dfa, PROCESS_CMD_START);
    defState(dfa, PROCESS_CMD_S10);
    defInput(dfa, PROCESS_CMD_DELETED);

    // where to transition after ending the load sms flow -- to be set upon entering the flow
    Dfa::State loadSmsNextState = IDLE; // needs initialization here
    int loadSmsRetryCount;

    void loadSmsCommands();

    UEventLoopTimer sim7000CallbackTimer;

    struct {
        struct {
            bool isSleepDurationAuto;
            int sleepDurationMillis;
        } shuttingDown;
    } stateData;

    SystemService *systemService;
    RebootDetectorService *rebootDetectorService;
    EEPROMFsService *eepromFsService;
    RtcPcf8563Service *rtc;
    Sim7000Service *sim7000;
    ButtonService::SingleButtonService *button;
    WifiAsyncService *wifi;
    Ina3221Service *ina3221;
    LM75A_TempService *tempService;
#ifdef USE_AM2320
    AM2320Service *am2320;
#endif
#ifdef USE_BLE
    BleService *ble;
#endif
#ifdef USE_ESP32I2C
    Esp32I2cService *esp32i2c;
#endif
#ifdef USE_LED_INDICATOR
    LedIndicatorService *ledIndicator;
#endif

    UEventLoopTimer wifiTimer;

    int buttonPin;
    int loadEnablePin;
    int loadOnPin;
    int crowbarPin;
    bool isEnableLoad;
    int startupDelayMillis; // delay from startup to looking for incoming SMS
    int autoShutdownDelayMillis;
    bool isAutoShutdown;
    bool isExtendedAutoShutdown;
    int64_t extendedAutoShutdownStartTs; // timestamp when extended auto shutdown started -- it can restart again, e.g. if button is pressed again
    int autoShutdownExtendedDelayMillis;
    int sleepDurationSecs;
    /** second minute hour dayOfMonth month dayOfWeek */
    String loadCron;
    int loadDurationMillis;
    bool isTelemetryEnabled;
    int telemetryUploadSize; // if telemetry reaches this size, upload it
    int telemetryClearSize; // if telemetry reaches this size, clear it (should be greater than telemetryUploadSizze)
    int telemetryMinPeriodSecs;
    bool isTelemetryInProcess;

    IvanSupervisorLedIndicators ledIndicators;

    String sim7000Msg;
    EEPROMLittleFs *eepromFs;

    Dfa loadFlow;
    defState(loadFlow, LOAD_INIT);
    defState(loadFlow, LOAD_START);
    defState(loadFlow, LOAD_TURN_ON);
    defState(loadFlow, LOAD_WORKING);
    defState(loadFlow, LOAD_TURNED_OFF);
    defState(loadFlow, LOAD_DONE);

    defInput(loadFlow, LOAD_STOP);
    defInput(loadFlow, LOAD_REQUEST_START);
    defInput(loadFlow, LOAD_UPLOAD_TERMINATED);


    bool isLoadLaunched; // whether the main DFA launched the load flow
    bool isNoLoadWakeup; // do not wake up according to load schedule -- set internally, not a command
    bool isNoWakeupShutdown; // do not program wakeup, shutdown will be definitive
    int loadStartTs;
    int64_t startupTs;
    bool isShutdownCancelled;
    bool loadFlowFinished;
    int failedShutdownCount;
    bool isLoadExecuted; // whether we executed the load during this wakeup
    UEventLoopTimer onPinTimer; // used to send a 1 Hz square signal on loadOnPin, for testing
    int pinOnPulseStatus; // used to send a 1 Hz square signal on loadOnPin, for testing

    Dfa commFlow;

    int calcSleepDurationMillis();
    // see whether load need to run
    bool calcRunLoad();

    struct {
        std::deque<String> commands;
        String pumpCron;
        bool isStopRequested;
        bool needsConfigSave;
        /** commands executed so far, reset to 0 after a telemetry upload */
        int commandsExecutedCount;
    } smsCmd;

    /** recordType is always 2 chars */
    void telemetry(const char *recordType, const char *dataType, int32_t data);
    void telemetry(const char *recordType,
        const char *dataType1 = nullptr, const char *data1 = nullptr,
        const char *dataType2 = nullptr, const char *data2 = nullptr,
        const char *dataType3 = nullptr, const char *data3 = nullptr,
        const char *dataType4 = nullptr, const char *data4 = nullptr,
        const char *dataType5 = nullptr, const char *data5 = nullptr,
        const char *dataType6 = nullptr, const char *data6 = nullptr,
        const char *dataType7 = nullptr, const char *data7 = nullptr,
        const char *dataType8 = nullptr, const char *data8 = nullptr,
        const char *dataType9 = nullptr, const char *data9 = nullptr,
        const char *dataType10 = nullptr, const char *data10 = nullptr
    );
    void telemetryAndPersistent(const char *recordType, const char *dataType, int32_t data);
    void telemetryAndPersistent(const char *recordType,
        const char *dataType1 = nullptr, const char *data1 = nullptr,
        const char *dataType2 = nullptr, const char *data2 = nullptr,
        const char *dataType3 = nullptr, const char *data3 = nullptr,
        const char *dataType4 = nullptr, const char *data4 = nullptr,
        const char *dataType5 = nullptr, const char *data5 = nullptr,
        const char *dataType6 = nullptr, const char *data6 = nullptr,
        const char *dataType7 = nullptr, const char *data7 = nullptr,
        const char *dataType8 = nullptr, const char *data8 = nullptr,
        const char *dataType9 = nullptr, const char *data9 = nullptr,
        const char *dataType10 = nullptr, const char *data10 = nullptr
    );
    void doTelemetry(bool isPersistent, const char *recordType,
        const char *dataType1 = nullptr, const char *data1 = nullptr,
        const char *dataType2 = nullptr, const char *data2 = nullptr,
        const char *dataType3 = nullptr, const char *data3 = nullptr,
        const char *dataType4 = nullptr, const char *data4 = nullptr,
        const char *dataType5 = nullptr, const char *data5 = nullptr,
        const char *dataType6 = nullptr, const char *data6 = nullptr,
        const char *dataType7 = nullptr, const char *data7 = nullptr,
        const char *dataType8 = nullptr, const char *data8 = nullptr,
        const char *dataType9 = nullptr, const char *data9 = nullptr,
        const char *dataType10 = nullptr, const char *data10 = nullptr
    );
    // return false if there's no telemetry to do; else send UPLOAD_TERMINATED after telemetry finished (OK or not)
    bool initiateTelemetryUpload();
    RotatingFile persistentFile;

    Sensors sensors;

    // Persistent state: an offset where to write a value, in the
    // file "persistent.data" in eeprom fs.
    // Value is int32_t (or something else, to be taken in account in offsets)

    void setPersistentState(int state, int value);
    int getPersistentState(int state);
#define PERSISTENT_STATE_OVERCURRENT 0
#define PERSISTENT_STATE_CROWBAR (PERSISTENT_STATE_OVERCURRENT + 4)

    void protDoCrowbar();
    void protDoLoadOff();
    void protDoShutdown();
    void protDoShutdownNoWakeup();

};


#endif
#endif