#include <CompilationOpts.h>

#ifdef USE_IVAN_SUPERVISOR

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "RtcPcf8563.h"
#include "LogMgr.h"
#include "Util.h"
#include "Sim7000.h"
#include "ButtonService.h"
#include "hwcrypto/sha.h"
#include "IvanSupervisor.h"
#include <ccronexpr.h>

/*

Protections
-----------

Anytime when not running the load (and expecially just after shutting down the load),
if non-negligeable current is passing through the load switch
    kill the load switch fuse,
    store the error status,
    for next load runs, don't try to run the load if this status is set but send telemetry indicating this status instead

While running the load, if overcurrent
    stop running the load,
    send telemetry with the load run error
        if this repeats on next run, set a flag to indicate error and don't try to run the load next times
        if this does not repeat on next run, clear the flag

While running the load, if undercurrent
    send telemetry with the load run undercurrent situation

If shutdown fails after retries
    store the error status
    transmit in telemetry
    kill the main switch fuse (actually we don't have hardware for this...)

If enclosure temperature is too high, stop load running, or don't run it.
Send via telemetry as load run status.

*/

/*

Sensors
-------

Upon startup, measure temp and humidity. Measure 12V. Send via telemetry.

All the time, measure current through the load switch. (Serves for protection.)

While load is running, measure load current and voltage frequently,
quantize (with hysteresis?) + run-length encode, at the end send via telemetry.

All the time, measure enclosure temperature.
Send via telemetry the value upon startup and at the end (before shutdown).

*/




IvanSupervisorService::IvanSupervisorService()
: dfa("main", 1), loadFlow("loadFlow", 2)
{
}

void IvanSupervisorService::init(UEventLoop *eventLoop, CommandMgr *commandMgr,
        SystemService *systemService, RebootDetectorService *rebootDetectorService,
        ButtonService *buttonService, EEPROMFsService *eepromFsService, WifiAsyncService *wifi,
        RtcPcf8563Service *rtc, Sim7000Service *sim7000, Ina3221Service *ina3221, LM75A_TempService *tempService,
#ifdef USE_AM2320
        AM2320Service *am2320,
#endif
#ifdef USE_BLE
        BleService *ble,
#endif
#ifdef USE_ESP32I2C
        Esp32I2cService *esp32i2c,
#endif
#ifdef USE_LED_INDICATOR
        LedIndicatorService *ledIndicator,
#endif
        LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("supervisor");
    this->systemService = systemService;
    this->rebootDetectorService = rebootDetectorService;
    this->eepromFsService = eepromFsService;
    this->rtc = rtc;
    this->sim7000 = sim7000;
    this->button = buttonService->getButton(0);
    this->wifi = wifi;
    this->ina3221 = ina3221;
    this->tempService = tempService;
#ifdef USE_AM2320
    this->am2320 = am2320;
#endif
#ifdef USE_BLE
    this->ble = ble;
#endif
#ifdef USE_ESP32I2C
    this->esp32i2c = esp32i2c;
#endif
#ifdef USE_LED_INDICATOR
    this->ledIndicator = ledIndicator;
#endif

    eepromFs = eepromFsService->getFs(); // eepromFs can be null, if eeprom fs not initialized

    ServiceCommands *cmd = commandMgr->getServiceCommands("supervisor");
    initDfa();
    initLoadFlow();
    initCommands(cmd);

    sensors.init(this, cmd);

    eventLoop->registerTimer(&rtcTimer);
    eventLoop->registerTimer(&sim7000CallbackTimer);
    eventLoop->registerTimer(&onPinTimer);

    // defaults
    buttonPin = -1;
    loadEnablePin = -1;
    loadOnPin = -1;
    crowbarPin = -1;
    isAutoShutdown = false;
    isEnableLoad = false;
    startupDelayMillis = 45000;
    autoShutdownDelayMillis = 30000;
    isExtendedAutoShutdown = false;
    autoShutdownExtendedDelayMillis = 10 * 60 * 1000;
    sleepDurationSecs = 2 * 60 * 60;
    loadDurationMillis = 2 * 60 * 1000;
    loadCron = "never";
    isTelemetryEnabled = false;
    telemetryUploadSize = 1200;
    telemetryClearSize = 3600;
    telemetryMinPeriodSecs = 24 * 3600;
    isTelemetryInProcess = false;

    isLoadLaunched = false;
    isNoLoadWakeup = false;
    isNoWakeupShutdown = false;
    isShutdownCancelled = false;
    loadFlowFinished = false;
    failedShutdownCount = 0;
    isLoadExecuted = false;
    sim7000Msg.clear();
    smsCmd.commands.clear();
    smsCmd.isStopRequested = false;
    smsCmd.pumpCron.clear();
    smsCmd.needsConfigSave = false;
    smsCmd.commandsExecutedCount = 0;

    persistentFile.openForWrite("/persistent.txt", eepromFs, 1000);

    // init data structures

    ledIndicators.init(eventLoop, cmd, ledIndicator, logger);

    // load config
    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    // init hardware

    rtc->clockDisable();
    rtc->timerEnable(5 * 60 * 1000, RtcPcf8563Service::TIMER_ROUND_UP); // timer on 5 minutes, just in case we shut down before setting something reasonable for next time
    rtcTimer.setCallback([this](UEventLoopTimer *timer) {
        if (dfa.getState() != SHUTTING_DOWN_S1 && dfa.getState() != SHUTTING_DOWN_S2) {
            this->rtc->timerEnable(5 * 60 * 1000, RtcPcf8563Service::TIMER_ROUND_UP); // 5 minutes more
            rtcTimer.setTimeout(2 * 60 * 1000);
        }
    });
    rtcTimer.setTimeout(2 * 60 * 1000);

    button->enable();
    button->onEvent(ButtonService::BUTTON_ON, [this](int id, ButtonService::ButtonEvent event, bool alreadyHandled) {
        logger->debug("Button ON");

        // program rtc not to wake up - will have to revert if BUTTON_OFF without a BUTTON_LONG_CLICK
        // need to do now, in case BUTTON_LONG_CLICK never happens because of hardware shutdown
        rtcTimer.cancelTimeout();
        this->rtc->clockDisable();
        this->rtc->timerDisable();

        return true;
    });
    button->onEvent(ButtonService::BUTTON_OFF, [this](int id, ButtonService::ButtonEvent event, bool alreadyHandled) {
        logger->debug("Button OFF");
        if (!isNoWakeupShutdown) {
            // re-arm rtc, since we're not shutting down
            this->rtc->timerEnable(5 * 60 * 1000, RtcPcf8563Service::TIMER_ROUND_UP); // as during init
            rtcTimer.setTimeout(2 * 60 * 1000);
        }
        return true;
    });
    button->onEvent(ButtonService::BUTTON_CLICK, [this](int id, ButtonService::ButtonEvent event, bool alreadyHandled) {
        logger->debug("Button CLICK");

        if (this->wifi->getStatus() == WifiAsyncService::WifiStatus::STOPPED) {
            this->wifi->startWifi();
        }
        isExtendedAutoShutdown = true;
        extendedAutoShutdownStartTs = millis();
        return true;
    });
    button->onEvent(ButtonService::BUTTON_LONG_CLICK, [this](int id, ButtonService::ButtonEvent event, bool alreadyHandled) {
        logger->debug("Button LONG_CLICK");

        logger->info("Shutting down on button long press with no wakeup programmed");

        // no need to stop the load if already running, because the load stops if the switch is not powered anymore

        // program rtc not to wake up
        rtcTimer.cancelTimeout();
        this->rtc->clockDisable();
        this->rtc->timerDisable();
        telemetry(TELEMETRY_SHUTDOWN_MANUALLY);

        // in case the button is released before shutting down, queue a shutdown command
        isNoWakeupShutdown = true;
        stateData.shuttingDown.isSleepDurationAuto = false; // not used
        stateData.shuttingDown.sleepDurationMillis = 0; // not used
        isShutdownCancelled = false;
        isExtendedAutoShutdown = false;
        dfa.queueInputForState(REQUEST_SHUTDOWN, IDLE); // what if we're in the shutdown process, waiting for timeout? TODO

        return true;
    });

    pinMode(loadEnablePin, INPUT_PULLUP);
    pinMode(loadEnablePin, OUTPUT);
    digitalWrite(loadEnablePin, HIGH);

    pinMode(loadOnPin, INPUT_PULLDOWN);
    pinMode(loadOnPin, OUTPUT);
    digitalWrite(loadOnPin, LOW);

    if (crowbarPin != -1) {
        pinMode(crowbarPin, INPUT_PULLDOWN);
        pinMode(crowbarPin, OUTPUT);
        digitalWrite(crowbarPin, LOW);
    }

    sensors.start();

    ledIndicators.setWithLoad(calcRunLoad());
    ledIndicators.setWithWifi(wifi->getStatus() == WifiAsyncService::CONNECTED
        || wifi->getStatus() == WifiAsyncService::STANDALONE);
    ledIndicators.setWithAutoShutdown(this->isAutoShutdown);
    ledIndicators.event(IvanSupervisorLedIndicators::LI_ON);
    this->wifi->onStart([this](bool isStandalone) {
        ledIndicators.setWithWifi(this->wifi->getStatus() == WifiAsyncService::CONNECTED
            || this->wifi->getStatus() == WifiAsyncService::STANDALONE);
    });

    dfa.handleInput(STARTUP_INITIATE);
}

void IvanSupervisorService::telemetry(const char *recordType, const char *dataType, int32_t data)
{
    char buf[20];
    itoa(data, buf, 10);
    telemetry(recordType, dataType, buf);
}
void IvanSupervisorService::telemetry(const char *recordType,
        const char *dataType1, const char *data1, const char *dataType2, const char *data2,
        const char *dataType3, const char *data3, const char *dataType4, const char *data4,
        const char *dataType5, const char *data5, const char *dataType6, const char *data6,
        const char *dataType7, const char *data7, const char *dataType8, const char *data8,
        const char *dataType9, const char *data9, const char *dataType10, const char *data10)
{
    doTelemetry(false, recordType, dataType1, data1, dataType2, data2, dataType3, data3,
        dataType4, data4, dataType5, data5, dataType6, data6, dataType7, data7, dataType8, data8, dataType9, data9, dataType10, data10);
}

void IvanSupervisorService::doTelemetry(bool isPersistent, const char *recordType,
        const char *dataType1, const char *data1, const char *dataType2, const char *data2,
        const char *dataType3, const char *data3, const char *dataType4, const char *data4,
        const char *dataType5, const char *data5, const char *dataType6, const char *data6,
        const char *dataType7, const char *data7, const char *dataType8, const char *data8,
        const char *dataType9, const char *data9, const char *dataType10, const char *data10)
{
    time_t ts = time(nullptr);

    DynamicJsonBuffer buf;
    JsonObject &o = buf.createObject();
    o["type"] = recordType;
    o["ts"] = ts;
    // we can use data<n> to construct the object, even though data won't be copied
    // into the JsonBuffer, because we'll print out the constructed object and discard
    // the JsonBuffer before data<n> go out of scope.
    if (dataType1 != nullptr) {
        o[dataType1] = data1;
    }
    if (dataType2 != nullptr) {
        o[dataType2] = data2;
    }
    if (dataType3 != nullptr) {
        o[dataType3] = data3;
    }
    if (dataType4 != nullptr) {
        o[dataType4] = data4;
    }
    if (dataType5 != nullptr) {
        o[dataType5] = data5;
    }
    if (dataType6 != nullptr) {
        o[dataType6] = data6;
    }
    if (dataType7 != nullptr) {
        o[dataType7] = data7;
    }
    if (dataType8 != nullptr) {
        o[dataType8] = data8;
    }
    if (dataType9 != nullptr) {
        o[dataType9] = data9;
    }
    if (dataType10 != nullptr) {
        o[dataType10] = data10;
    }
    size_t len = o.measureLength();
    char msgBuf[len + 1];
    o.printTo(msgBuf, sizeof(msgBuf));

    if (eepromFs == nullptr) {
        return;
    }
    logger->info("Telemetry {} {} {}", recordType,
        (uint32_t)ts, msgBuf);

    File logFile = eepromFs->open("/telemetry.txt", "a+");
    if (logFile) {
        logFile.println(msgBuf);
        logFile.flush();
        logFile.close();
    }

    if (isPersistent) {
        persistentFile.println(msgBuf);
        persistentFile.terminateRecord();
    }
}

void IvanSupervisorService::telemetryAndPersistent(const char *recordType, const char *dataType, int32_t data)
{
    doTelemetry(true, recordType, dataType, String(data).c_str());
}

void IvanSupervisorService::telemetryAndPersistent(const char *recordType,
        const char *dataType1, const char *data1,
        const char *dataType2, const char *data2,
        const char *dataType3, const char *data3,
        const char *dataType4, const char *data4,
        const char *dataType5, const char *data5,
        const char *dataType6, const char *data6,
        const char *dataType7, const char *data7,
        const char *dataType8, const char *data8,
        const char *dataType9, const char *data9,
        const char *dataType10, const char *data10)
{
    doTelemetry(true, recordType, dataType1, data1, dataType2, data2,
        data3, dataType3, data4, dataType4, data5, dataType5, data6, dataType6,
        data7, dataType7, data8, dataType8, data9, dataType9, data10, dataType10);
}

bool IvanSupervisorService::initiateTelemetryUpload()
{
    if (eepromFs == nullptr) {
        return false;
    }
    File logFile = eepromFs->open("/telemetry.txt", "a+");
    if (!logFile) {
        return false;
    }
    size_t size = logFile.size();
    logFile.close();
    if (size == 0) {
        return false;
    }
    if (isTelemetryInProcess) {
        return false;
    }
    isTelemetryInProcess = true;

    bool rc;
    eepromFs->remove("/telemetry.sending.txt");
    eepromFs->rename("/telemetry.txt", "/telemetry.sending.txt");
    if (this->isTelemetryEnabled) {
        // upload telemetry file
        logger->info("Telemetry uploading {} bytes", size);
        char *buf = new char[size + 1];
        File f = eepromFs->open("/telemetry.sending.txt", "a+");
        f.readBytes(buf, size);
        f.close();
        buf[size] = '\0';
        // if we end up clearing the data without sending it, we'd like to know
        // the number of lines, first and last timestamps, and number of
        // lines indicating wakeup (TELEMETRY_START).
        struct ClearedInfo {
            int bufLines = 0;
            int bufTelemetryStartCount = 0;
            int bufFirstTs = 0; // timestamp
            int bufLastTs = 0; // timestamp
        } clearedInfo;
        char *ptr = buf;
        DynamicJsonBuffer jsonBuf;
        String record;
        do {
            char *next = strstr("\n", ptr);
            if (next == nullptr) { // we're done (records are always terminated by '\n')
                break;
            }
            ++clearedInfo.bufLines;
            record.clear();
            // terminate current record with '\0', copy it to a string, then restore it
            char c = *next;
            *next = '\0';
            record.concat(ptr);
            *next = c;
            JsonObject &jsonRecord = jsonBuf.parseObject(record);
            if (jsonRecord.success()) {
                const char *type = jsonRecord["type"].as<char *>();
                if (type != nullptr && strcmp(type, TELEMETRY_START)) {
                    ++clearedInfo.bufTelemetryStartCount;
                }
                if (jsonRecord.containsKey("ts")) {
                    int ts = jsonRecord["ts"].as<int>();
                    if (clearedInfo.bufFirstTs == 0 || ts < clearedInfo.bufFirstTs) {
                        clearedInfo.bufFirstTs = ts;
                    }
                    if (clearedInfo.bufLastTs == 0 || ts < clearedInfo.bufLastTs) {
                        clearedInfo.bufLastTs = ts;
                    }
                }
            }
        } while (true);

        logger->debug("Telemetry uploading {} bytes: {}", size, buf);
        this->sim7000->initiateSendMessage(buf, buf, [this, size, &clearedInfo](void *arg, bool sent, bool confirmed) {
            logger->info("Telemetry uploading sent: {}, confirmed: {}",
                sent ? "true" : "false",
                confirmed ? "true" : "false");
            delete[] (char *)arg;

            // if the size of telemetry data still allows, we'll keep the old non-confirmed (or not sent)
            // data so that it is sent next time
            if (!confirmed && size < this->telemetryClearSize) {
                // we may already have a new telemetry.txt, as new telemetry has come in
                // append new telementry to old file, then rename
                bool oldFileDataCopied = false;
                fs::File newFile = eepromFs->open("/telemetry.txt");
                if (newFile && size + newFile.size() < this->telemetryClearSize ) { // we ask for the totality to fit in telemetryClearSize
                    char *buf = new char[newFile.size()];
                    int read = newFile.readBytes(buf, size);
                    newFile.close();
                    if (read > 0) {
                        fs::File oldFile = eepromFs->open("/telemetry.sending.txt", "a");
                        oldFile.write((uint8_t *)buf, read);
                        oldFile.close();
                        oldFileDataCopied = true;
                    }
                }
                eepromFs->rename("/telemetry.sending.txt", "/telemetry.txt");
                if (!oldFileDataCopied) {
                    telemetry(sent ? TELEMETRY_UPLOAD_SENT_NOT_CONFIRMED : TELEMETRY_UPLOAD_NOT_SENT,
                        TELEMETRY_UPLOAD_GSM_RSSI, String(sim7000->getRssi()).c_str(),
                        TELEMETRY_UPLOAD_SIM7000_ERROR, sim7000->getError());
                }
            } else {
                eepromFs->remove("/telemetry.sending.txt");
                if (confirmed) {
                    telemetry(TELEMETRY_UPLOAD_SENT_CONFIRMED);
                } else {
                    // we're about to clear old collected data

                    telemetryAndPersistent(sent ? TELEMETRY_UPLOAD_SENT_NOT_CONFIRMED_CLEARED : TELEMETRY_UPLOAD_NOT_SENT_CLEARED,
                        TELEMETRY_UPLOAD_GSM_RSSI, String(sim7000->getRssi()).c_str(),
                        TELEMETRY_UPLOAD_SIM7000_ERROR, sim7000->getError(),
                        TELEMETRY_CLEARED_LINES, String(clearedInfo.bufLines).c_str(),
                        TELEMETRY_CLEARED_START_COUNT, String(clearedInfo.bufTelemetryStartCount).c_str(),
                        TELEMETRY_CLEARED_FIRST_TS, String(clearedInfo.bufFirstTs).c_str(),
                        TELEMETRY_CLEARED_LAST_TS, String(clearedInfo.bufLastTs).c_str()
                    );
                }
            }

            isTelemetryInProcess = false;
            loadFlow.handleInput(LOAD_UPLOAD_TERMINATED);
        });

        smsCmd.commandsExecutedCount = 0; // resetting count of executed commands
        rc = true;

    } else { // else we're not removing .sending, as we're testing. It will be removed just before next upload.
        File f = eepromFs->open("/telemetry.sending.txt", "a");
        f.println("{\"type\":\"COMMENT\",\"comment\":\"Telemetry was disabled, no transmission.\"}");
        f.close();
        rc = false;
        isTelemetryInProcess = false;
    }

    File lastTelemetryFile = eepromFs->open("/telemetryLast.txt", "w");
    if (lastTelemetryFile) {
        time_t curTime = time(nullptr);
        char buf[30];
        ctime_r(&curTime, buf);
        buf[strlen(buf) - 1] = '\0';
        lastTelemetryFile.print(curTime, 10);
        lastTelemetryFile.print(" - ");
        lastTelemetryFile.print(buf);
        lastTelemetryFile.close();
    }
    return rc;
}

void IvanSupervisorService::loadSmsCommands()
{
    for (int i = 0; i < sim7000->getSmsCount(); i++) {
        Sim7000Service::Sms *sms = sim7000->getSms(i);
        if (!sms->isValid || sms->isToDelete) {
            continue;
        }

        logger->debug("Received command via SMS: {}", sms->text);
        char result[100];
        bool isError = false;
        result[0] = '\0';

        String text(sms->text);
        if (text.startsWith("CMD ")) {
            text.replace("CMD ", "");
            smsCmd.commands.push_back(text);
        } else if (text.startsWith("STOP ")) {
            text.replace("STOP ", "");
            char *endPtr;
            unsigned rq = strtoul(text.c_str(), &endPtr, 10);
            while (*endPtr == ' ' || *endPtr == '\t') {
                ++endPtr;
            }
            if ((rq != 0 && rq != 1) || *endPtr != '\0') {
                snprintf(result, sizeof(result), "Invalid STOP command: %s", text.c_str());
                isError = true;
            } else {
                smsCmd.isStopRequested = (rq == 1);
            }
        } else if (text.startsWith("PUMPD ")) {
            text.replace("PUMPD ", "");
            char *endPtr;
            unsigned d = strtoul(text.c_str(), &endPtr, 10);
            unsigned h = strtoul(endPtr, &endPtr, 10);
            while (*endPtr == ' ' || *endPtr == '\t') {
                ++endPtr;
            }
            if (d == 0 || d > 30 || h > 23 || *endPtr != '\0') {
                snprintf(result, sizeof(result), "Invalid PUMPD command: %s", text.c_str());
                isError = true;
            } else {
                smsCmd.pumpCron = String("supervisor loadCron 0 0 ") + h + " */" + d + " * *";
            }
        } else if (text.startsWith("PUMPH ")) {
            text.replace("PUMPH ", "");
            char *endPtr;
            unsigned h = strtoul(text.c_str(), &endPtr, 10);
            while (*endPtr == ' ' || *endPtr == '\t') {
                ++endPtr;
            }
            if (h > 23 || *endPtr != '\0') {
                snprintf(result, sizeof(result), "Invalid PUMPD command: %s", text.c_str());
                isError = true;
            } else {
                smsCmd.pumpCron = String("supervisor loadCron 0 0 */") + h + " * * *";
            }
        } else if (text.equals("PUMPNONE")) {
            smsCmd.pumpCron = String("supervisor loadCron never");
        } else {
            isError = true;
            snprintf(result, sizeof(result), "Unknown command via SMS: %s", text.c_str());
        }

        if (isError) {
            logger->error("Invalid command via SMS: \"{}\"", result);
            this->telemetry(TELEMETRY_CMD_PROCESSED_FAILURE,
                TELEMETRY_DATA_CMD_TEXT, text.c_str(),
                TELEMETRY_DATA_CMD_RESULT, result);
        }

        sim7000->markSmsForDeletion(i);
    }

    // commands in smsCmc.commands will be performed while dfa is in IDLE

    // add commands for pumpCron and isStopRequested
    if (!smsCmd.pumpCron.isEmpty()) {
        smsCmd.commands.push_back(smsCmd.pumpCron);
        smsCmd.pumpCron.clear();
        smsCmd.needsConfigSave = true;
    }
    if (smsCmd.isStopRequested) {
        smsCmd.commands.push_back(String("supervisor shutdown nowakeup"));
        smsCmd.isStopRequested = false;
    }
}

// 8888888b.  8888888888     d8888 
// 888  "Y88b 888           d88888 
// 888    888 888          d88P888 
// 888    888 8888888     d88P 888 
// 888    888 888        d88P  888 
// 888    888 888       d88P   888 
// 888  .d88P 888      d8888888888 
// 8888888P"  888     d88P     888 

void IvanSupervisorService::initDfa()
{
    dfa.init(eventLoop, logger, STARTUP);

    dfa.onInput([this](Dfa *dfa, Dfa::State state, Dfa::Input input) {

        if (state.is(STARTUP)) {

            if (input.is(STARTUP_INITIATE)) {
                startupTs = millis();
                if (rebootDetectorService->isInvoluntaryShutdown()) {
                    telemetryAndPersistent(TELEMETRY_START, TELEMETRY_DATA_INVOLUNTARY_SHUTDOWN, 1);
                    logger->warn("Last shutdown was involuntary!");
                } else {
                    telemetry(TELEMETRY_START);
                }

                // wait a bit after startup, to give time to sim7000 to start up and receive any SMS
                dfa->setStateTimeout(startupDelayMillis);
                return dfa->noTransition();
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                loadSmsNextState = IDLE;
                return dfa->transitionTo(LOAD_SMS_START);
            } else {
                return dfa->transitionError();
            }

        /**
         * Load SMS
         **/
        } else if (state.is(LOAD_SMS_START)) { // expect loadSmsNextState to contain the state to go to after loading SMSs.
            if (input.is(Dfa::Input::ENTER_STATE)) {
                loadSmsRetryCount = 0;
                return dfa->transitionTo(LOAD_SMS_S01);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(LOAD_SMS_S01)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                bool didInitiate = sim7000->initiateLoadSmss([this](bool isSuccess) {
                    sim7000CallbackTimer.setCallback([this, isSuccess](UEventLoopTimer *timer) {
                        this->dfa.handleInput(isSuccess ? LOAD_SMS_LOAD_SUCCEEDED : LOAD_SMS_LOAD_FAILED);
                    });
                    sim7000CallbackTimer.setTimeout(1);
                });
                if (didInitiate) {
                    return dfa->transitionTo(LOAD_SMS_S10);
                } else {
                    // wait some more time, at TIMEOUT we'll retry
                    logger->warn("Error loading SMSs, retry {}", loadSmsRetryCount);
                    return dfa->transitionTo(LOAD_SMS_RETRY);
                }
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(LOAD_SMS_RETRY)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                dfa->setStateTimeout(1000);
                return dfa->noTransition();
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                ++loadSmsRetryCount;
                if (loadSmsRetryCount < 5) {
                    return dfa->transitionTo(LOAD_SMS_S01);
                } else {
                    return dfa->transitionTo(LOAD_SMS_TERMINATED);
                }
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(LOAD_SMS_S10)) {

            if (input.is(LOAD_SMS_LOAD_SUCCEEDED)) {
                loadSmsCommands();
                if (smsCmd.commands.size() == 0 && sim7000->getSmsToDeleteCount() > 0) {
                    // we need to delete the SMSs now; otherwise, the SMSs will be
                    // deleted after processing all commands, when smsCmd.commands.size() == 0
                    sim7000->deleteMarkedSms([this](int deletionsRequested, int deletionsPerformed) {
                        this->dfa.queueInput(LOAD_SMS_DELETED, 1);
                    });
                    return dfa->transitionTo(LOAD_SMS_S20);
                } else {
                    return dfa->transitionTo(LOAD_SMS_TERMINATED);
                }
            } else if (input.is(LOAD_SMS_LOAD_FAILED)) {
                logger->debug("Failed to load SMS list");
                this->telemetry(TELEMETRY_CMD_RETRIEVE_FAILURE);
                return dfa->transitionTo(LOAD_SMS_TERMINATED);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(LOAD_SMS_S20)) {

            if (input.is(LOAD_SMS_DELETED)) {
                return dfa->transitionTo(LOAD_SMS_TERMINATED);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(LOAD_SMS_TERMINATED)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                return dfa->transitionTo(loadSmsNextState);
            } else {
                return dfa->transitionError();
            }

        /**
         * IDLE
         **/

        } else if (state.is(IDLE)) {

            if (input.is(Dfa::Input::ENTER_STATE, Dfa::Input::TIMEOUT)) {
                // in all cases, we execute any pending commands first
                if (smsCmd.commands.size() > 0) {
                    return dfa->transitionTo(PROCESS_CMD_START);
                } else if (!isLoadLaunched) {
                    isLoadLaunched = true;
                    loadFlow.handleInput(LOAD_REQUEST_START);
                    dfa->setStateTimeout(1000); // so we'll time out, and re-enter this block via input TIMEOUT
                    return dfa->noTransition();
                } else if (loadFlowFinished && isAutoShutdown && failedShutdownCount < 5) {
                    // will auto shutdown either when entering IDLE when load flow is finished,
                    // or when load flow finishes when the state is IDLE
                    stateData.shuttingDown.isSleepDurationAuto = true;
                    return dfa->transitionTo(DO_SHUTDOWN_WITH_WAIT);
                } else {
                    // Nothing more to do. Could set a timeout, if we need to do something
                    return dfa->noTransition();
                }
            } else if (input.is(REQUEST_LOAD_SMS)) {
                loadSmsNextState = IDLE;
                return dfa->transitionTo(LOAD_SMS_START);
            } else if (input.is(REQUEST_SHUTDOWN_WITH_WAIT)) {
                stateData.shuttingDown.isSleepDurationAuto = true;
                return dfa->transitionTo(DO_SHUTDOWN_WITH_WAIT);
            } else if (input.is(REQUEST_SHUTDOWN)) {
                return dfa->transitionTo(DO_SHUTDOWN);
            } else {
                return dfa->transitionError();
            }

        /**
         * Process one received command
         **/
        } else if (state.is(PROCESS_CMD_START)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                if (smsCmd.commands.size() > 0) { // have a little timeout before processing a command, so that command processing is not synchronous
                    String *cmd = &smsCmd.commands[0];
                    if (cmd->startsWith("DELAY ")) { // if it's DELAY, we wait for the specified duration
                        String delayStr = *cmd;
                        delayStr.replace("DELAY ", "");
                        delayStr.trim();
                        char *endPtr;
                        unsigned delay = strtoul(delayStr.c_str(), &endPtr, 10);
                        if (*endPtr != '\0') { // something is wrong with the syntax
                            delay = 1;
                        }
                        if (delay < 1 || delay > 60 * 1000) { // delay no more than 60 seconds
                            delay = 10000;
                        }
                        logger->debug("Processing command {} with delay = {}", cmd->c_str(), delay);
                        dfa->setStateTimeout(delay);
                    } else {
                        int delay = 1;
                        logger->debug("Processing command {} with delay = {}", cmd->c_str(), delay);
                        dfa->setStateTimeout(delay);
                    }
                    return dfa->noTransition();
                } else {
                    return dfa->transitionTo(IDLE);
                }
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                String *cmd = &smsCmd.commands[0];
                if (!cmd->startsWith("DELAY ")) { // DELAY has already been processed, using timeout
                    logger->debug("Processing command \"{}\"", cmd->c_str());
                    String cmdRet(*cmd);
                    bool isProcessed = commandMgr->processCommandLine("SMS", &cmdRet);
                    if (isProcessed) {
                        this->telemetry(TELEMETRY_CMD_PROCESSED_SUCCESSFULLY,
                            TELEMETRY_DATA_CMD_TEXT, cmd->c_str(),
                            TELEMETRY_DATA_CMD_RESULT, cmdRet.c_str());
                    } else {
                        this->telemetry(TELEMETRY_CMD_PROCESSED_FAILURE, TELEMETRY_DATA_CMD_TEXT, cmd->c_str());
                    }
                    logger->debug("Processed command \"{}\", processed: {}, result: \"{}\"",
                        cmd->c_str(),
                        isProcessed ? "true" : "false",
                        cmdRet.c_str()
                    );
                }
                smsCmd.commands.pop_front();
                ++smsCmd.commandsExecutedCount;

                if (smsCmd.commands.size() == 0) { // save config, clear smsCmd, delete processed SMSs
                    if (smsCmd.needsConfigSave) {
                        // must save config, as commands may have changed it
                        String cmd("supervisor save");
                        commandMgr->processCommandLine("SMS", &cmd);
                    }
                    smsCmd.commands.clear();
                    smsCmd.isStopRequested = false;
                    smsCmd.pumpCron.clear();
                    smsCmd.needsConfigSave = false;

                    sim7000->deleteMarkedSms([this](int deletionsRequested, int deletionsPerformed) {
                        this->dfa.handleInput(PROCESS_CMD_DELETED);
                    });
                    return dfa->transitionTo(PROCESS_CMD_S10);
                } else {
                    return dfa->transitionTo(IDLE);
                }
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(PROCESS_CMD_S10)) {

            if (input.is(PROCESS_CMD_DELETED)) {
                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }
        /**
         * Shutdown
         **/
        } else if (state.is(DO_SHUTDOWN_WITH_WAIT)) {

            // calculate wait duration, go to DO_SHUTDOWN_AFTER_WAIT with timeout
            if (input.is(Dfa::Input::ENTER_STATE)) {
                int requestedExtendedDelay = 0;
                if (isExtendedAutoShutdown) {
                    int64_t extendedAwakeTime = millis() - extendedAutoShutdownStartTs;
                    if (extendedAwakeTime < 0) {
                        extendedAwakeTime = INT64_MAX;
                    }
                    if (extendedAwakeTime < autoShutdownExtendedDelayMillis) {
                        requestedExtendedDelay = autoShutdownExtendedDelayMillis - extendedAwakeTime;
                    }
                }

                int requestedDelay = 0;
                int64_t timeSinceStartup = millis() - startupTs;
                if (timeSinceStartup < 0) { // we've wrapped over
                    timeSinceStartup = INT64_MAX;
                }
                if (timeSinceStartup < autoShutdownDelayMillis) {
                    requestedDelay = autoShutdownDelayMillis - timeSinceStartup;
                }

                int beforeShutdown = (requestedExtendedDelay > requestedDelay ? requestedExtendedDelay  : requestedDelay);
                if (beforeShutdown == 0) {
                    beforeShutdown = 1;
                }
                return dfa->transitionTo(DO_SHUTDOWN_AFTER_WAIT, beforeShutdown);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(DO_SHUTDOWN_AFTER_WAIT)) {

            if (input.is(Dfa::Input::TIMEOUT)) {
                return dfa->transitionTo(DO_SHUTDOWN);
            } else if (input.is(CANCEL_SHUTDOWN)) {
                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(DO_SHUTDOWN)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {

                if (isShutdownCancelled) {
                    // isAutoShutdown must have been set to false, otherwise we'll start the shutdown sequence again
                    ledIndicators.event(IvanSupervisorLedIndicators::LI_LOAD_DONE);
                    return dfa->transitionTo(IDLE);
                }

                ledIndicators.event(IvanSupervisorLedIndicators::LI_SHUTTING_DOWN);
                // If we're now in extended auto shutdown, check whether we spend enough time awake.
                // If not, go back to IDLE
                if (isExtendedAutoShutdown) {
                    int extendedAwakeTime = millis() - extendedAutoShutdownStartTs;
                    if (extendedAwakeTime < 0) {
                        extendedAwakeTime = INT_MAX;
                    }
                    if (extendedAwakeTime < autoShutdownExtendedDelayMillis) {
                        return dfa->transitionTo(IDLE);
                    }
                }
                // else we go ahead with shutdown

                // program the RTC for the wakeup
                rtcTimer.cancelTimeout();
                if (isNoWakeupShutdown) {
                    // will not wake up
                    rtc->clockDisable();
                    rtc->timerDisable();
                    stateData.shuttingDown.sleepDurationMillis = -1; // used in telemetry
                    logger->info("Shutting down with no wakeup programmed");
                } else {
                    // program next wakeup
                    rtc->clockEnable(1);
                    if (stateData.shuttingDown.isSleepDurationAuto) {
                        stateData.shuttingDown.sleepDurationMillis = calcSleepDurationMillis();
                    }
                    uint32_t actualTime = rtc->timerEnable(stateData.shuttingDown.sleepDurationMillis, RtcPcf8563Service::TIMER_ROUND_UP);
                    logger->info("Set timeout at {} millis, actual {}, will shut down",
                        stateData.shuttingDown.sleepDurationMillis, actualTime);
                }

                return dfa->transitionTo(SHUTTING_DOWN_S1, 1000);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SHUTTING_DOWN_S1)) {

            if (input.is(Dfa::Input::TIMEOUT)) {
                ledIndicators.event(IvanSupervisorLedIndicators::LI_SHUTTING_DOWN);
                // Set pin to command the main switch
                if (isNoWakeupShutdown) {
                    telemetryAndPersistent(TELEMETRY_SHUTDOWN, TELEMETRY_DATA_SLEEP_DURATION, stateData.shuttingDown.sleepDurationMillis);
                } else {
                    telemetry(TELEMETRY_SHUTDOWN, TELEMETRY_DATA_SLEEP_DURATION, stateData.shuttingDown.sleepDurationMillis);
                }
                systemService->setWillShutDown();

                button->disable();
                pinMode(buttonPin, OUTPUT);
                digitalWrite(buttonPin, HIGH);

                // in case we don't shut down in 20 seconds, we'll reset the pin state
                return dfa->transitionTo(SHUTTING_DOWN_S2, 30000);

            } else if (input.is(CANCEL_SHUTDOWN)) {
                pinMode(buttonPin, INPUT);
                button->enable();

                rtc->clockDisable();
                rtc->timerEnable(5 * 60 * 1000, RtcPcf8563Service::TIMER_ROUND_UP);
                rtcTimer.setTimeout(2 * 60 * 1000);
                ledIndicators.event(IvanSupervisorLedIndicators::LI_LOAD_DONE);
                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SHUTTING_DOWN_S2)) {

            if (input.is(Dfa::Input::TIMEOUT, CANCEL_SHUTDOWN)) {
                // We didn't manage to shut down...
                // what to do now? Log something...? reboot ?
                // TODO
                ledIndicators.event(IvanSupervisorLedIndicators::LI_LOAD_DONE);
                telemetryAndPersistent(TELEMETRY_SHUTDOWN_NOT_PERFORMED);
                ++failedShutdownCount;

                pinMode(buttonPin, INPUT);
                button->enable();
                delay(200); // just so that no button click happens immediately now

                rtc->clockDisable();
                rtc->timerEnable(5 * 60 * 1000, RtcPcf8563Service::TIMER_ROUND_UP);
                rtcTimer.setTimeout(2 * 60 * 1000);

                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }

        } else {
            return dfa->transitionError();
        }

    });
}

bool IvanSupervisorService::calcRunLoad()
{
    // see if it's time to activate the load
    time_t cur = time(nullptr);
    time_t lastLoad; // set to 0 if we don't have a lastLoad, and we run immediately
    // see if we have lastLoad in the EEPROM file
    if (eepromFs != nullptr) {
        File file = eepromFs->open("/lastLoad.txt", "r");
        if (!file) { // we don't have a last execution time, do run now
            lastLoad = 0; // do run
        } else {
            String line = file.readString();
            lastLoad = strtol(line.c_str(), nullptr, 10);
            // leave lastLoad to 0 if it didn't parse well, behaving as if we didn't have a lastLoad
        }
        file.close();
    } else { // if no eeprom, don't run
        lastLoad = cur;
    }

    time_t nextLoad;
    bool runLoad;
    if (loadCron == "never") {
        runLoad = false;
    } else if (loadCron == "always") {
        runLoad = true;
    } else {
        if (lastLoad == 0) {
            runLoad = true;
        } else {
            cron_expr expr;
            const char *err;
            cron_parse_expr(loadCron.c_str(), &expr, &err);
            nextLoad = cron_next(&expr, lastLoad);
            runLoad = (difftime(nextLoad, cur) < 30); // if we're past the time of next load, or up to 30 seconds before
        }
    }
    return runLoad;
}

// 888                            888      8888888888 888                             8888888b.  8888888888     d8888 
// 888                            888      888        888                             888  "Y88b 888           d88888 
// 888                            888      888        888                             888    888 888          d88P888 
// 888      .d88b.   8888b.   .d88888      8888888    888  .d88b.  888  888  888      888    888 8888888     d88P 888 
// 888     d88""88b     "88b d88" 888      888        888 d88""88b 888  888  888      888    888 888        d88P  888 
// 888     888  888 .d888888 888  888      888        888 888  888 888  888  888      888    888 888       d88P   888 
// 888     Y88..88P 888  888 Y88b 888      888        888 Y88..88P Y88b 888 d88P      888  .d88P 888      d8888888888 
// 88888888 "Y88P"  "Y888888  "Y88888      888        888  "Y88P"   "Y8888888P"       8888888P"  888     d88P     888 

void IvanSupervisorService::initLoadFlow()
{
    loadFlow.init(eventLoop, logger, LOAD_INIT);
    isLoadExecuted = false;
    loadFlowFinished = false;

    loadFlow.onInput([this](Dfa *loadFlow, Dfa::State state, Dfa::Input input) {
        if (state.is(LOAD_INIT)) {

            if (input.is(LOAD_REQUEST_START)) {
                ledIndicators.setWithLoad(calcRunLoad());
                return loadFlow->transitionTo(LOAD_START);
            } else {
                return loadFlow->transitionError();
            }

        } else if (state.is(LOAD_START)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                // read sensors
                int mainVoltage, mainCurrent, loadVoltage, loadCurrent,
                    caseTemp, envTemp, envHumidity, rssi, sensorBatteryPct, sensorRssi;
                sensors.readSensors(&mainVoltage, &mainCurrent, &loadVoltage, &loadCurrent,
                    &caseTemp, &envTemp, &envHumidity, &sensorBatteryPct, &sensorRssi, &rssi);
                telemetry(TELEMETRY_SENSOR_DATA,
                    TELEMETRY_SENSOR_DATA_MAIN_VOLTAGE, String(mainVoltage).c_str(),
                    TELEMETRY_SENSOR_DATA_MAIN_CURRENT, String(mainCurrent).c_str(),
                    TELEMETRY_SENSOR_DATA_LOAD_VOLTAGE, String(loadVoltage).c_str(),
                    TELEMETRY_SENSOR_DATA_LOAD_CURRENT, String(loadCurrent).c_str(),
                    TELEMETRY_SENSOR_DATA_CASE_TEMP, String(caseTemp).c_str(),
                    TELEMETRY_SENSOR_DATA_GSM_RSSI, String(rssi).c_str(),
                    TELEMETRY_SENSOR_DATA_ENV_TEMP, String(envTemp).c_str(),
                    TELEMETRY_SENSOR_DATA_ENV_HUMIDITY, String(envHumidity).c_str(),
                    TELEMETRY_SENSOR_DATA_ENV_BATTERY_PCT, String(sensorBatteryPct).c_str(),
                    TELEMETRY_SENSOR_DATA_ENV_RSSI, String(sensorRssi).c_str()
                );
                loadFlowFinished = false;
                isLoadExecuted = false;
                // see if we have work to do
                bool runLoad = calcRunLoad();
                ledIndicators.setWithLoad(runLoad);
                if (isEnableLoad) {                    
                    // Before activating the load, we must make sure we'll be able to write to disk
                    // the new value in /lastLoad.txt. If impossible because of disk full, we'll
                    // run the load every time.
                    // For this, keeping in mind that LittleFs is a copy-on-write file system, we need to create a
                    // placeholder file just to reserve some space, and remove it just before
                    // writing into /lastLoad.txt.
                    bool spaceFileOk;
                    if (eepromFs != nullptr) {
                        eepromFs->remove("/space.txt");
                        File spaceFile = eepromFs->open("/space.txt", "w");
                        spaceFileOk = false;
                        if (spaceFile) {
                            char buf[50];
                            memset(buf, 'X', sizeof(buf) - 1);
                            buf[sizeof(buf) - 1] = '\0';
                            size_t w = spaceFile.print(buf); // just some data with a length greater than what /lastLoad.txt would have
                            spaceFileOk = (w > 0);
                            spaceFile.close();
                            if (spaceFile.getWriteError() < 0) {
                                spaceFileOk = false;
                            }
                        }
                    } else {
                        spaceFileOk = true;
                    }

                    if (!spaceFileOk) {
                        logger->error("Couldn't write file /space.txt to EEPROM, couldn't reserve space for /lastLoad.txt, load will not run");
                        runLoad = false;
                        // store this fact, so when we shut down, we know we shouldn't
                        // take load execution to wake up
                        isNoLoadWakeup = true;
                    } else {
                        isNoLoadWakeup = false;
                    }

                    if (runLoad) {
                        // activate the load switch
                        digitalWrite(loadEnablePin, LOW);
                        delay(1);
                        digitalWrite(loadEnablePin, HIGH);
                        delay(1);
                        return loadFlow->transitionTo(LOAD_TURN_ON);
                    } else {
                        return loadFlow->transitionTo(LOAD_DONE);
                    }

                } else {
                    return loadFlow->transitionTo(LOAD_DONE);
                }
            }

        } else if (state.is(LOAD_TURN_ON)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                loadFlow->setStateTimeout(2000); // turn on in 2 seconds TODO 1s?
                return loadFlow->noTransition();
            } else if (input.is(LOAD_STOP)) {
                return loadFlow->transitionTo(LOAD_DONE); // without setting isLoadExecuted
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                // write down lastLoad
                time_t curTime = time(nullptr);
                char buf[30];
                ctime_r(&curTime, buf);
                buf[strlen(buf) - 1] = '\0';

                // /lastLoad.txt is critical, so we've reserved space for it, which we free now
                if (eepromFs != nullptr) {
                    eepromFs->remove("/space.txt");
                    File file = eepromFs->open("/lastLoad.txt", "w");
                    file.print(curTime, 10);
                    file.print(" - ");
                    file.println(buf);
                    file.close();
                }

                return loadFlow->transitionTo(LOAD_WORKING);
            }

        } else if (state.is(LOAD_WORKING)) {

            if (input.is(Dfa::Input::ENTER_STATE)) { // activate

                // first ensure we've a sensor check right now
                sensors.spotMeasure();
                if (sensors.protIsDoNotLoad()) {
                    // don't turn on the load
                    telemetry(TELEMETRY_LOAD_BEGIN_FORBIDDEN);
                    int mainVoltage, mainCurrent, loadVoltage, loadCurrent,
                        caseTemp, envTemp, envHumidity, rssi, sensorBatteryPct, sensorRssi;
                    sensors.readSensors(&mainVoltage, &mainCurrent, &loadVoltage, &loadCurrent,
                        &caseTemp, &envTemp, &envHumidity, &sensorBatteryPct, &sensorRssi, &rssi);
                    telemetry(TELEMETRY_SENSOR_DATA,
                        TELEMETRY_SENSOR_DATA_MAIN_VOLTAGE, String(mainVoltage).c_str(),
                        TELEMETRY_SENSOR_DATA_MAIN_CURRENT, String(mainCurrent).c_str(),
                        TELEMETRY_SENSOR_DATA_LOAD_VOLTAGE, String(loadVoltage).c_str(),
                        TELEMETRY_SENSOR_DATA_LOAD_CURRENT, String(loadCurrent).c_str(),
                        TELEMETRY_SENSOR_DATA_CASE_TEMP, String(caseTemp).c_str(),
                        TELEMETRY_SENSOR_DATA_GSM_RSSI, String(rssi).c_str(),
                        TELEMETRY_SENSOR_DATA_ENV_TEMP, String(envTemp).c_str(),
                        TELEMETRY_SENSOR_DATA_ENV_HUMIDITY, String(envHumidity).c_str(),
                        TELEMETRY_SENSOR_DATA_ENV_BATTERY_PCT, String(sensorBatteryPct).c_str(),
                        TELEMETRY_SENSOR_DATA_ENV_RSSI, String(sensorRssi).c_str()
                    );
                    return loadFlow->transitionTo(LOAD_DONE);
                } else {
                    isLoadExecuted = true;
                    ledIndicators.event(IvanSupervisorLedIndicators::LI_LOAD_RUNNING);
                    telemetry(TELEMETRY_LOAD_BEGIN);
                    // telemetry before turning on, because it takes time, and we want minimum time between turn on and a LOAD_STOP
                    sensors.loadOnOvercurrentSupervisionPrepare();
                    digitalWrite(loadOnPin, HIGH);
                    sensors.setLoadState(true);
                    int overcurrent = sensors.loadOnOvercurrentSupervision(30); // n millis close supervision for overcurrent (blocking call)
                    if (overcurrent > 0) {
                        digitalWrite(loadOnPin, LOW); // as quickly as possible
                        telemetry(TELEMETRY_LOAD_END, TELEMETRY_LOAD_END_ABORTED_OVERCURRENT, overcurrent);
                        return loadFlow->transitionTo(LOAD_TURNED_OFF);
                    } else {
                        loadFlow->setStateTimeout(loadDurationMillis);
                        return loadFlow->noTransition();
                    }
                }
            } else if (input.is(Dfa::Input::TIMEOUT) || input.is(LOAD_STOP)) {
                // turn off
                digitalWrite(loadOnPin, LOW);
                telemetry(TELEMETRY_LOAD_END);
                ledIndicators.event(IvanSupervisorLedIndicators::LI_LOAD_DONE);
                return loadFlow->transitionTo(LOAD_TURNED_OFF);
            }

        } else if (state.is(LOAD_TURNED_OFF)) { // we'll wait a bit after turning off, then read sensors and terminate

            if (input.is(Dfa::Input::ENTER_STATE)) {
                sensors.setLoadState(false);
                loadFlow->setStateTimeout(1000);
                return loadFlow->noTransition();
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                // read sensors
                String loadProfile;
                sensors.readLoadProfile(&loadProfile);
                telemetry(TELEMETRY_SENSOR_LOAD_PROFILE, TELEMETRY_SENSOR_LOAD_PROFILE_VALUE, loadProfile.c_str());

                int mainVoltage, mainCurrent, loadVoltage, loadCurrent,
                    caseTemp, envTemp, envHumidity, rssi, sensorBatteryPct, sensorRssi;
                sensors.readSensors(&mainVoltage, &mainCurrent, &loadVoltage, &loadCurrent,
                    &caseTemp, &envTemp, &envHumidity, &sensorBatteryPct, &sensorRssi, &rssi);
                telemetry(TELEMETRY_SENSOR_DATA,
                    TELEMETRY_SENSOR_DATA_MAIN_VOLTAGE, String(mainVoltage).c_str(),
                    TELEMETRY_SENSOR_DATA_MAIN_CURRENT, String(mainCurrent).c_str(),
                    TELEMETRY_SENSOR_DATA_LOAD_VOLTAGE, String(loadVoltage).c_str(),
                    TELEMETRY_SENSOR_DATA_LOAD_CURRENT, String(loadCurrent).c_str(),
                    TELEMETRY_SENSOR_DATA_CASE_TEMP, String(caseTemp).c_str(),
                    TELEMETRY_SENSOR_DATA_GSM_RSSI, String(rssi).c_str(),
                    TELEMETRY_SENSOR_DATA_ENV_TEMP, String(envTemp).c_str(),
                    TELEMETRY_SENSOR_DATA_ENV_HUMIDITY, String(envHumidity).c_str(),
                    TELEMETRY_SENSOR_DATA_ENV_BATTERY_PCT, String(sensorBatteryPct).c_str(),
                    TELEMETRY_SENSOR_DATA_ENV_RSSI, String(sensorRssi).c_str()
                );
                return loadFlow->transitionTo(LOAD_DONE);
            }

        } else if (state.is(LOAD_DONE)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {

                // check to see if we need telemetry upload
                // do an upload if we ran the load, or if the file size is big enough
                if (eepromFs != nullptr) {
                    File f = eepromFs->open("/telemetry.txt");
                    size_t size = 0;
                    if (f) {
                        size = f.size();
                        f.close();
                    }
                    time_t lastUpload = 0;
                    File lastTelemetryFile = eepromFs->open("/telemetryLast.txt");
                    if (lastTelemetryFile) {
                        String str = lastTelemetryFile.readString();
                        char *end;
                        lastUpload = strtol(str.c_str(), &end, 10); // results in 0 if can't parse
                        lastTelemetryFile.close();
                    }

                    if (isLoadExecuted
                            || smsCmd.commandsExecutedCount > 0 // if we executed commands, we do telemetry upload
                            || (lastUpload != 0 && difftime(lastUpload + this->telemetryMinPeriodSecs, time(nullptr)) < 0)
                            || size > telemetryUploadSize) {
                        bool rc = initiateTelemetryUpload();
                        // upon upload it will send UPLOAD_TERMINATED
                        if (!rc) {
                            loadFlow->setStateTimeout(10000); // if there's no upload (and no UPLOAD_TERMINATED), time out to finish
                        }
                    } else {
                        loadFlow->setStateTimeout(10000); // Wait a bit after finishing load. TODO set to 1s.
                    }
                } else {
                    loadFlow->setStateTimeout(10000);
                }
                return loadFlow->noTransition();
            } else if (input.is(LOAD_UPLOAD_TERMINATED)) {
                loadFlow->setStateTimeout(1000); // Wait a bit
                return loadFlow->noTransition();
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                ledIndicators.event(IvanSupervisorLedIndicators::LI_LOAD_DONE);
                loadFlowFinished = true;
                // will auto shutdown either when entering IDLE when load flow is finished,
                // or when laod flow finishes when the state is IDLE
                if (isAutoShutdown) {
                    dfa.queueInputForState(REQUEST_SHUTDOWN_WITH_WAIT, IDLE);
                }
                return loadFlow->transitionTo(LOAD_DONE);
            } else if (input.is(LOAD_REQUEST_START)) {
                return loadFlow->transitionTo(LOAD_START);
            }

        }

        return loadFlow->transitionError();
    });

}

void IvanSupervisorService::protDoCrowbar()
{
    digitalWrite(crowbarPin, HIGH);
}

void IvanSupervisorService::protDoLoadOff()
{
    if (loadFlow.getState() == LOAD_WORKING) {
        loadFlow.handleInput(LOAD_STOP);
    } else {
        // just in case -- but we should never see a loadOnPin to HIGH if the state is not LOAD_WORKING
        // and when in LOAD_WORKING, the dfa will always handle input LOAD_STOP.
        digitalWrite(loadOnPin, LOW);
        loadFlow.queueInputForState(LOAD_STOP, LOAD_WORKING);
    }
}

void IvanSupervisorService::protDoShutdown()
{
    isNoWakeupShutdown = false;
    stateData.shuttingDown.isSleepDurationAuto = true;
    stateData.shuttingDown.sleepDurationMillis = 0; // not used, because isSleepDurationAuto = true
    isShutdownCancelled = false;
    isExtendedAutoShutdown = false;
    dfa.queueInputForState(REQUEST_SHUTDOWN, IDLE); // what if we're in the shutdown process, waiting for timeout? TODO
}

void IvanSupervisorService::protDoShutdownNoWakeup()
{
    isNoWakeupShutdown = true;
    stateData.shuttingDown.isSleepDurationAuto = false; // not used
    stateData.shuttingDown.sleepDurationMillis = 0; // not used
    isShutdownCancelled = false;
    isExtendedAutoShutdown = false;
    isShutdownCancelled = false;
    dfa.queueInputForState(REQUEST_SHUTDOWN, IDLE); // what if we're in the shutdown process, waiting for timeout? TODO
}

void IvanSupervisorService::setPersistentState(int state, int value)
{
    fs::File f = eepromFs->open("persistent.data", "w+");
    bool rc = false;
    if (f) {
        rc = f.seek(state);
    }
    if (rc) {
        int32_t val = value;
        f.write((uint8_t *)&val, 4);
    }
    if (f) {
        f.close();
    }
}

int IvanSupervisorService::getPersistentState(int state)
{
    fs::File f = eepromFs->open("persistent.data", "r");
    bool rc = false;
    int32_t val = 0;
    if (f) {
        rc = f.seek(state);
    }
    if (rc) {
        f.readBytes((char *)&val, 4);
    }
    if (f) {
        f.close();
    }
    return val;
}



//  .d8888b.                                                              888          
// d88P  Y88b                                                             888          
// 888    888                                                             888          
// 888         .d88b.  88888b.d88b.  88888b.d88b.   8888b.  88888b.   .d88888 .d8888b  
// 888        d88""88b 888 "888 "88b 888 "888 "88b     "88b 888 "88b d88" 888 88K      
// 888    888 888  888 888  888  888 888  888  888 .d888888 888  888 888  888 "Y8888b. 
// Y88b  d88P Y88..88P 888  888  888 888  888  888 888  888 888  888 Y88b 888      X88 
//  "Y8888P"   "Y88P"  888  888  888 888  888  888 "Y888888 888  888  "Y88888  88888P' 



void IvanSupervisorService::initCommands(ServiceCommands *cmd)
{
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("buttonPin", true)
        .cmd("buttonPin")
        .help("--> Set button pin, requires save and reboot. Normally low, button pulls it high, MCU pulls it high for 20 sec to shut down.")
        .vMin(-1)
        .vMax(99)
        .ptr(&buttonPin)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("loadEnablePin", true)
        .cmd("loadEnablePin")
        .help("--> Set load enable pin, requires save and reboot. The pin is default high, will be pulsed low (for 1ms) at startup of load run.")
        .vMin(-1)
        .vMax(99)
        .ptr(&loadEnablePin)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("loadOnPin", true)
        .cmd("loadOnPin")
        .help("--> Set load on pin, requires save and reboot. The pin will be held high for the duration of the load work.")
        .vMin(-1)
        .vMax(99)
        .ptr(&loadOnPin)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("crowbarPin", true)
        .cmd("crowbarPin")
        .help("--> Set crowbar pin.")
        .vMin(-1)
        .vMax(99)
        .ptr(&crowbarPin)
        .setFn([this] (int val, bool isLoading, String *msg) {
            if (isLoading) {
                crowbarPin = val;
                return true;
            }
            if (crowbarPin != -1 && val != -1) {
                pinMode(crowbarPin, INPUT);
            }
            crowbarPin = val;
            if (crowbarPin != -1) {
                pinMode(crowbarPin, INPUT_PULLDOWN);
                pinMode(crowbarPin, OUTPUT);
                digitalWrite(crowbarPin, LOW);
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("startupDelay", true)
        .cmd("startupDelay")
        .help("--> Delay in seconds after startup, before looking for received SMS.")
        .vMin(0)
        .vMax(60 * 60000)
        .setFn([this](int val, int isLoading, String *msg) {
            startupDelayMillis = 1000 * val;
            *msg = "Startup delay set to "; msg->concat(val); msg->concat(" seconds");
            return true;
        })
        .getFn([this]() {
            return startupDelayMillis / 1000;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("autoShutdown", true)
        .cmd("autoShutdown")
        .help("--> If on, automatic shutdown will be performed once all tasks are done.")
        .ptr(&isAutoShutdown)
        .setFn([this](bool val, bool isLoading, String *msg) {
            isAutoShutdown = val;
            if (isAutoShutdown) { // limit values for delays
                if (autoShutdownDelayMillis < 10000) { // min 10 seconds
                    autoShutdownDelayMillis = 10000;
                } else if (autoShutdownDelayMillis > 10 * 60 * 1000) { // max 10 min
                    autoShutdownDelayMillis = 10 * 60 * 1000;
                }
                if (autoShutdownExtendedDelayMillis < 30000) { // min 30 seconds
                    autoShutdownExtendedDelayMillis = 30000;
                } else if (autoShutdownExtendedDelayMillis > 30 * 60 * 1000) { // max 30 min
                    autoShutdownExtendedDelayMillis = 30 * 60 * 1000;
                }
                if (loadFlowFinished) {
                    // reset the startup timestamp, so we start counting for autoshutdown from now
                    startupTs = millis();
                    dfa.queueInputForState(REQUEST_SHUTDOWN_WITH_WAIT, IDLE);
                }
                isShutdownCancelled = false;
            } else {
                isShutdownCancelled = true;
                dfa.handleInput(CANCEL_SHUTDOWN); // may be processed or not
            }
            *msg = "Set auto shutdown to ";
            *msg += (isAutoShutdown ? "true" : "false");
            if (isAutoShutdown) {
                *msg += ", delay "; *msg += autoShutdownDelayMillis;
                *msg += ", extended delay "; *msg += (int)autoShutdownExtendedDelayMillis;
            }
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("autoShutdownDelay", true)
        .cmd("autoShutdownDelay")
        .help("--> Delay before shutdown after auto shutdown is started. In millis, or can suffix with 's', 'm'.")
        .setFn([this](const String &val, int isLoading, String *msg) {
            int v = getDuration(val, false, msg);
            if (v != 0) {
                if (v < 10000) {
                    *msg = "Minimum value is 10s, minimum value set";
                    v = 10000;
                } else if (v > 10 * 60 * 1000) {
                    *msg = "Maximum value is 10m, maximum value set";
                    v = 10 * 60 * 1000;
                }
                autoShutdownDelayMillis = v;
            }
            return true;
        })
        .getFn([this](String *val) {
            *val = String(autoShutdownDelayMillis);
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("autoShutdownExtendedDelay", true)
        .cmd("autoShutdownExtendedDelay")
        .help("--> Delay before shutdown after auto shutdown is started, if the button is pressed. In millis, or can suffix with 's', 'm'.")
        .setFn([this](const String &val, int isLoading, String *msg) {
            int v = getDuration(val, false, msg);
            if (v != 0) {
                if (v < 30000) {
                    *msg = "Minimum value is 30s, minimum value set";
                    v = 30000;
                } else if (v > 30 * 60 * 1000) {
                    *msg = "Maximum value is 30m, maximum value set";
                    v = 30 * 60 * 1000;
                }
                autoShutdownExtendedDelayMillis = v;
            }
            return true;
        })
        .getFn([this](String *val) {
            *val = String(autoShutdownExtendedDelayMillis);
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("sleepDuration", true)
        .cmd("sleepDuration")
        .help("--> Duration of sleep between two wakeups. In seconds, or can suffix with 'm', 'h'.")
        .setFn([this](const String &val, int isLoading, String *msg) {
            int v = getDuration(val, true, msg);
            if (v != 0) {
                if (v < 30) {
                    *msg = "Minimum value is 30s (as about 10s are needed to perform shutdown), minimum value set";
                    v = 30;
                } else if (v > 255 * 60) {
                    *msg = "Maximum value is 255m (limit of the RTC), maximum value set";
                    v = 255 * 60;
                }
                sleepDurationSecs = v;
            }
            return true;
        })
        .getFn([this](String *val) {
            *val = String(sleepDurationSecs);
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("loadCron", true)
        .cmd("loadCron")
        .help("--> Cron-like expression for load activation, or \"always\" or \"never\". Format:\n"
            "    <sec> <min> <hour> <day of month> <month> <day of week>\n"
            "    <n>-<m>: range, <n>,<m>: list, *: any, ?: ignore, /<n>: every <n>")
        .ptr(&loadCron)
        .setFn([this](const String &val, int isLoading, String *msg) {
            if (val == "never") {
                loadCron = "never";
            } else if (val == "always") {
                loadCron = "always";
            } else {
                cron_expr expr;
                const char *err = nullptr;
                cron_parse_expr(val.c_str(), &expr, &err);
                if (err != nullptr) {
                    *msg = "Error parsing cron expression \"" + val + "\": " + err;
                    return true;
                }
                loadCron = val;
            }
            *msg = "Value of loadCron set to ";
            *msg += loadCron;
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("loadEnable", true)
        .cmd("loadEnable")
        .help("--> Enable or disable driving the load switch.")
        .ptr(&isEnableLoad)
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("loadDuration", true)
        .cmd("loadDuration")
        .help("--> Duration of load working. In millis, or can suffix with 's', 'm', 'h'.")
        .setFn([this](const String &val, int isLoading, String *msg) {
            int v = getDuration(val, false, msg);
            if (v != 0) {
                if (v < 1) {
                    *msg = "Minimum value is 1 millisecond, minimum value set";
                    v = 1;
                } else if (v > 5 * 60 * 1000) {
                    *msg = "Maximum value is 5m, maximum value set";
                    v = 5 * 60 * 1000;
                }
                loadDurationMillis = v;
            }
            return true;
        })
        .getFn([this](String *val) {
            *val = String(loadDurationMillis);
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("load", true) // attn: load is already a command (global one)
        .cmdOn("loadStart")
        .helpOn("--> Start the load switch process, according to loadCron definition.")
        .cmdOff("loadStop")
        .helpOff("--> Immediately stop the load switch")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (val) {
                if (loadFlow.getState() == LOAD_DONE) {
                    loadFlow.handleInput(LOAD_REQUEST_START);
                    *msg = "Load restarting";
                } else {
                    *msg = "Load flow not in LOAD_DONE state";
                }
            } else {
                if (loadFlow.getState() == LOAD_WORKING) {
                    loadFlow.handleInput(LOAD_STOP);
                    *msg = "Load stopping";
                } else {
                    *msg = "Load flow not in WORKING state";
                }
            }
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("loadPinEnable", true)
        .cmdOn("loadPinEnable")
        .helpOn("--> [Low-level] Pulse once the loadEnable pin")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            digitalWrite(loadEnablePin, LOW);
            delay(1);
            digitalWrite(loadEnablePin, HIGH);
            delay(1);
            *msg = "Load enable pin was pulsed";
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("loadPinOn", true)
        .cmd("loadPinOn")
        .help("--> [Low-level] Turn on or off the loadOn pin")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            digitalWrite(loadOnPin, val ? HIGH : LOW);
            *msg = "Pin loadOn is set to "; msg->concat(val ? "HIGH" : "LOW");
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("crowbar", true)
        .cmd("crowbar")
        .help("crowbar on|off --> [Low-level] Turn on or off the crowbar pin")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (crowbarPin == -1) {
                *msg = "Crowbar pin is -1, command ignored";
                return true;
            }
            digitalWrite(crowbarPin, val ? HIGH : LOW);
            *msg = "Crowbar pin is set to "; msg->concat(val ? "HIGH" : "LOW");
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("loadPinOnPulse", true)
        .cmd("loadPinOnPulse")
        .help("loadPinOnPulse <n> --> [Low-level] Turn on or off pulse on the loadOn pin, <n> is pulse duration in millis, 0 turns off")
        .isPersistent(false)
        .vMin(0)
        .setFn([this](int val, bool isLoading, String *msg) {
            if (val > 0) {
                pinOnPulseStatus = LOW;
                onPinTimer.setInterval([this](UEventLoopTimer *timer) {
                    pinOnPulseStatus = (pinOnPulseStatus == LOW ? HIGH : LOW);
                    digitalWrite(loadOnPin, pinOnPulseStatus);
                }, val);
                *msg = "Pin loadOn will pulse for "; msg->concat(val);
                msg->concat(" millis");
            } else {
                onPinTimer.cancelInterval();
                pinOnPulseStatus = LOW;
                digitalWrite(loadOnPin, LOW);
                *msg = "Pin loadOn set to LOW, pulsing stopped";
            }
            return true;
        })
    );


    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("dfaState", true)
        .cmd("dfaState")
        .help("--> DFA state")
        .isPersistent(false)
        .getFn([this](String *val) {
            *val = "";
            val->concat(dfa.getState().getId());
            val->concat("-");
            val->concat(dfa.stateName(dfa.getState()));
            val->concat(" since "); Util::durationToStr(val, dfa.getMillisInState());
            val->concat(", timeout: ");
            if (dfa.getStateTimeout() <= 0) {
                val->concat("none");
            } else {
                Util::durationToStr(val, dfa.getStateTimeout());
            }
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("loadFlowState", true)
        .cmd("loadFlowState")
        .help("--> Load flow DFA state")
        .isPersistent(false)
        .getFn([this](String *val) {
            *val = "";
            val->concat(loadFlow.getState().getId());
            val->concat("-");
            val->concat(loadFlow.stateName(loadFlow.getState()));
            val->concat(" since "); Util::durationToStr(val, loadFlow.getMillisInState());
            val->concat(", timeout: ");
            if (loadFlow.getStateTimeout() <= 0) {
                val->concat("none");
            } else {
                Util::durationToStr(val, loadFlow.getStateTimeout());
            }
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("sendMsg", true)
        .cmd("sendMsg")
        .help("--> Send a message to Hologram cloud")
        .setFn([this](const String &val, bool isLoading, String *msg) {
            if (sim7000->isSendingMessage()) {
                *msg = "Sending in process";
                return true;
            }
            sim7000Msg = val; // must not change sim7000 until sending is over
            sim7000->initiateSendMessage(sim7000Msg.c_str(), nullptr, [this](void *arg, bool sent, bool confirmed) {
                logger->info("Sent message to Hologram cloud, sent: {}, confirmed: {}", sent, confirmed);
                sim7000Msg.clear();
            });
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("cmdProcess", true)
        .cmdOn("cmdProcess")
        .helpOn("--> Start checking (and processing) incomming commands")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            dfa.queueInputForState(REQUEST_LOAD_SMS, IDLE);
            *msg = "Requested loading of SMS commands";
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("telemetry", true)
        .cmd("telemetry")
        .help("--> Enable or disable telemetry")
        .ptr(&isTelemetryEnabled)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("telemetryUploadSize", true)
        .cmd("telemetryUploadSize")
        .help("--> Once the telemetry file reaches this size, it will be uploaded")
        .ptr(&telemetryUploadSize)
        .vMin(1)
        .vMax(10000)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("telemetryClearSize", true)
        .cmd("telemetryClearSize")
        .help("--> If the telemetry file reaches this size and it's not uploaded yet, it will be cleared")
        .ptr(&telemetryClearSize)
        .vMin(1)
        .vMax(20000)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("telemetryPeriod", true)
        .cmd("telemetryPeriod")
        .help("--> Period in seconds between to planned telemetry uploads (independently of uploads for reaching max size)")
        .ptr(&telemetryMinPeriodSecs)
        .vMin(1)
        .vMax(60 * 60 * 24 * 5)
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("telemetryUpload", true)
        .cmdOn("telemetryUpload")
        .helpOn("--> Upload telemetry now.")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            this->initiateTelemetryUpload();
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("shutdown", true)
        .cmd("shutdown")
        .help("--> shutdown <n>: set wakeup rtc timer at <n> millis, then command the main switch\n"
            "    switch to shut down. Suffixes 's', 'm' and 'h' can be used. Maximum is 256 minutes.\n"
            "    Use \"shutdown cancel\" to cancel a shutdown.\n"
            "    Use \"shutdown nowakeup\" to shut down without programming the rtc timer for wakeup.")
        .isPersistent(false)
        .setFn([this](const String &val, bool isLoading, String *msg) {
            if (val == "cancel") {
                isShutdownCancelled = true;
                dfa.handleInput(CANCEL_SHUTDOWN); // may be processed or not
                *msg = "Shutdown (current or future) cancelled";
                return true;
            }

            bool requestShutdown;
            uint32_t delay;
            if (val == "nowakeup") {
                delay = 0;
                isNoWakeupShutdown = true;
                stateData.shuttingDown.isSleepDurationAuto = false; // not used
                stateData.shuttingDown.sleepDurationMillis = 0; // not used
                isShutdownCancelled = false;
                isExtendedAutoShutdown = false;
                requestShutdown = true;
            } else {
                delay = getDuration(val, false, msg);
                if (delay > 0) {
                    isNoWakeupShutdown = false;
                    stateData.shuttingDown.isSleepDurationAuto = false;
                    stateData.shuttingDown.sleepDurationMillis = delay;
                    isShutdownCancelled = false;
                    isExtendedAutoShutdown = false;
                    requestShutdown = true;
                } else {
                    requestShutdown = false;
                    *msg = "Error parsing wakeup duration, shutdown not performed";
                }
            }

            if (requestShutdown) {
                isShutdownCancelled = false;
                dfa.queueInputForState(REQUEST_SHUTDOWN, IDLE); // what if we're in the shutdown process, waiting for timeout? TODO
                if (isNoWakeupShutdown) {
                    *msg = "Requested shutdown with no wakeup";
                } else {
                    *msg = "Timer set at ";
                    *msg += delay;
                    *msg += String(" millis, starting shutdown sequence");
                }
            }
            return true;
        })
    );

}

int IvanSupervisorService::calcSleepDurationMillis()
{
    cron_expr expr;
    const char* err = nullptr;
    time_t cur = time(nullptr);
    time_t lastLoad;

    if (!isEnableLoad || isNoLoadWakeup) {
        return sleepDurationSecs * 1000;
    }

    // see if we have lastLoad in the EEPROM file
    if (eepromFs != nullptr) {
        File file = eepromFs->open("/lastLoad.txt", "r");
        if (!file) { // we don't have a last execution time, calculate from current time
            lastLoad = cur;
        } else {
            String line = file.readString();
            lastLoad = strtol(line.c_str(), nullptr, 10);
            if (lastLoad == 0) { // couldn't read the lastLoad from file
                lastLoad = cur;
            }
        }
        file.close();
    } else {
        lastLoad = cur;
    }
    unsigned long toSleep;

    if (loadCron.isEmpty() || loadCron.equals("never")) {
        toSleep = LONG_MAX;
    } else {
        // we're assuming cron_parse_expr doesn't fail, because we've checked the value
        // of sleepCron and loadCron
        cron_parse_expr(loadCron.c_str(), &expr, &err);
        time_t nextLoad = cron_next(&expr, lastLoad);
        long d = (long)difftime(nextLoad, cur);
        if (d < 0) { // we're already too late
            toSleep = 0;
        } else {
            toSleep = d;
        }
    }

    // adjust taking in account sleepDurationSecs
    if (toSleep < sleepDurationSecs) {
        if (toSleep < 30) { // we leave ourselves at least 30 seconds to shut down
            toSleep = 30;
        }
    } else if (toSleep < sleepDurationSecs + sleepDurationSecs / 2) {
        toSleep = toSleep / 2; // so that after one period we don't find ourselves with a very short sleep period
    } else {
        toSleep = sleepDurationSecs;
    }

    if (toSleep > 250 * 60) { // 250 minutes (max is 256)
        toSleep = 250 * 60;
    }
    return (int)toSleep * 1000;
}

uint32_t IvanSupervisorService::getDuration(const String &val, bool isSeconds /* or millis */, String *msg)
{
    uint32_t count;
    int last = 0; // trim end of string (beginning is trimmed by cmd)
    while (last < val.length() && val.charAt(last) >= '0' && val.charAt(last) <= '9') {
        ++last;
    }
    if (last == val.length()) {
        count = val.toInt();
        if (count <= 0) {
            *msg = String("Must provide duration in ") + (isSeconds ? "seconds" : "milliseconds") + ", > 0";
        }
    } else {
        char lastChar = val.charAt(last);
        count = val.toInt();
        int millisMult = (isSeconds ? 1 : 1000);
        switch (lastChar) {
            case 'h': count *= millisMult * 60 * 60; break;
            case 'm': count *= millisMult * 60; break;
            case 's': count *= millisMult; break;
            default: {
                *msg = "Unrecognized number suffix: \"%c\", only \"h\", \"m\" or \"s\" expected";
                count = 0;
            }
        }
    }

    return count;
}

#endif
