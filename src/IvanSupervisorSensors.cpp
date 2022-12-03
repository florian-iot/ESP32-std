#include <CompilationOpts.h>

#ifdef USE_IVAN_SUPERVISOR

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "IvanSupervisor.h"


Sensors::Sensors()
{

}

void Sensors::init(IvanSupervisorService *sup, ServiceCommands *cmd)
{
    this->sup = sup;

    initCommands(cmd);

    highFrequencyMillis = 10;
    lowFrequencyMillis = 1000;
    hystCurrentPct = 2;
    hystVoltagePct = 2;

    protIsEnabled = false;
    protCrowbarIsEnabled = false;
    protOffMaxCurrent = 10;
    protOffMaxVoltage = 10;
    protOnMaxCurrent = 5000; // 5A
    protOnMinVoltage = 10000;
    protMinVoltageToLoad = 12400;
    protMinVoltageShutdown = 12100;
    protMaxTempToLoad = 40000;
    protMaxTempShutdown = 50000;

    isLoadOn = false;
    loadOnTs = millis() - 3600000; // we've turned on since a long ago, for initialization
    loadOffTs = loadOnTs;
    isTurnOffInitiated = false;
    isShutdownInitiated = false;
    protIsOutputPowered = false;
    protIsCrowbar = false;
    protIsUndervoltage = false;
    protIsLowOutputVoltage = false;
    protIsOvertemp = false;
    protIsTempError = false;

    loadVoltage = 0; // mV
    loadCurrent = 0; // mA
    mainVoltage = 0; // mV
    mainCurrent = 0; // mA
    caseTemp = 0; // milli°C

    isFirstProfile = true;
    lastProfileTs = loadOffTs;
    lastLoadCurrent = 0;
    lastLoadVoltage = 0;
    measurementCounter = 0;

    timer.init(sup->eventLoop, [this](UEventLoopTimer *timer) {
        unsigned long ts = millis();
        bool isHighFreq = (isLoadOn || (!isLoadOn && ts - loadOffTs <= 5000));

        doSensorMeasurements(isHighFreq);

        // if we're on, or if we've turned off less than 5 seconds ago, we measure
        // at high rate. Else at low rate.
        unsigned long ts2 = millis();
        if (isHighFreq) {
            int timeout = highFrequencyMillis - (ts2 - ts);
            timer->setTimeout(timeout > 0 ? timeout : 1);
        } else {
            int timeout = lowFrequencyMillis - (ts2 - ts);
            timer->setTimeout(timeout > 0 ? timeout : 1);
        }
    });

#ifdef USE_AM2320
    sup->am2320->startMeasuring();
#endif
#ifdef USE_BLE
    sup->ble->startSensorScan();
#endif
}

void Sensors::start()
{
    timer.setTimeout(lowFrequencyMillis);
}

void Sensors::setLoadState(bool isOn)
{
    if (isOn && !isLoadOn) {
        isLoadOn = true;
        loadOnTs = millis();
        timer.cancelTimeout();
        timer.setTimeout(1); // when starting the load, we'll measure sensors 1ms after turning on, then at high frequency
        isTurnOffInitiated = false;

        // reset state (should not have to, but helps when protIsEnabled == false)
        protIsOutputPowered = false;
        protIsCrowbar = false;
        protIsUndervoltage = false;
        protIsLowOutputVoltage = false;

    } else if (!isOn && isLoadOn) {
        isLoadOn = false;
        loadOffTs = millis();
    }
}

void Sensors::varintAdd(std::vector<uint8_t> &vect, int32_t n, bool isSignedEncoding)
{
    if (isSignedEncoding) {
        n = (n << 1) ^ ((int32_t)n >> 31);
    }
    while (n & ~0x7F) {
        vect.push_back((n & 0xFF) | 0x80);
        n = n >> 7;
    }
    vect.push_back(n);
}

int32_t Sensors::varintDecode(std::vector<uint8_t> &vect, std::vector<uint8_t>::iterator &ptr, bool isSignedEncoding)
{
    uint32_t result = 0;
    uint32_t v;
    int b = 0;
    do {
        if (ptr == vect.end()) {
            Serial.printf("At %d, found end of vector\n", (int)(ptr - vect.begin()));
            return 0;
        }
        v = *ptr;
        result += ((v & 0x7F) << b);
        ptr++;
        b += 7;
    } while ((v & 0x80) != 0);
    if (isSignedEncoding) {
        int32_t signmask = (result & 1) ? -1 : 0;
        result = (result >> 1) ^ signmask;
    }
    return result;
}


void Sensors::loadOnOvercurrentSupervisionPrepare()
{
    sup->ina3221->setModeOvercurrent(2);
}

int Sensors::loadOnOvercurrentSupervision(int duration)
{
    int cnt = 0;
    int tm = millis();
    while (millis() - tm < duration) {
        sup->ina3221->readChannelData(2, &loadVoltage, &loadCurrent);
        if (loadCurrent > protOnMaxCurrent) {
            sup->logger->trace("Overcurrent supervision for {} millis, overcurrent {} mA, performed {} samplings",
                duration, loadCurrent, cnt);
            return loadCurrent;
        }
        delayMicroseconds(400); // give enough time for a cycle of measuring
        ++cnt;
    }
    sup->ina3221->setModeNormal();
    sup->logger->debug("Overcurrent supervision for {} millis, performed {} samplings", duration, cnt);
    return 0;
}

void Sensors::doSensorMeasurements(bool isHighFreq)
{
    ++measurementCounter;
    // int ts1 = millis();
    sup->ina3221->readChannelData(3, &mainVoltage, &mainCurrent);
    mainCurrent = -mainCurrent; // main current is inverted, we want it positive
    sup->ina3221->readChannelData(2, &loadVoltage, &loadCurrent);
    caseTemp = sup->tempService->readTempMilliC();
    int ts = millis();

    int hystCurrentMin = 20;
    int hystVoltageMin = 20;

    int currentHyst = abs(lastLoadCurrent) * (int)(hystCurrentPct * 100) / 10000;
    if (currentHyst < hystCurrentMin) {
        currentHyst = hystCurrentMin;
    }
    int voltageHyst = abs(lastLoadVoltage) * (int)(hystVoltagePct * 100) / 10000;
    if (voltageHyst < hystVoltageMin) {
        voltageHyst = hystVoltageMin;
    }

    bool currentOrVoltageChanged = // changed more than 20 mA / 20 mV and more than a percentage of last value
            (abs(loadCurrent - lastLoadCurrent) > currentHyst)
            || (abs(loadVoltage - lastLoadVoltage) > voltageHyst);

    // Protection conditions

    if (!isLoadOn && ts - loadOffTs > 5 && loadCurrent > protOffMaxCurrent) {
        // Current is still high after we turn off -- output mosfet failure, crowbar
        if (!protIsCrowbar) {
            sup->telemetryAndPersistent(TELEMETRY_CROWBAR, TELEMETRY_PROT_VALUE, loadCurrent);
            sup->setPersistentState(PERSISTENT_STATE_CROWBAR, 1);
            protIsCrowbar = true;
            sup->logger->error("Protection: Overcurrent while load is off: {} mA, turning on crowbar", loadCurrent);
            if (protCrowbarIsEnabled) { // we have a separate flag for enabling crowbar protection
                sup->protDoCrowbar();
            }
            // we'll most likey shut off
            // no time to send telemetry
            goto endOfConditions; // don't use the previous measurements -- anyways, here we are probably shut off
        }
    }
    if (!isLoadOn && ts - loadOffTs > 5 && loadCurrent <= protOffMaxCurrent) {
        // This is mostly for testing, if we're out of overcurrent condition. Normally
        // the crowbar would shut down the mcu
        if (protIsCrowbar) {
            sup->telemetryAndPersistent(TELEMETRY_NO_CROWBAR, TELEMETRY_PROT_VALUE, loadCurrent);
            sup->setPersistentState(PERSISTENT_STATE_CROWBAR, 0);
            protIsCrowbar = false;
            sup->logger->info("Protection: No longer overcurrent while load is off: {} mA", loadCurrent);
        }
    }

    if (!isLoadOn && ts - loadOffTs > 5 && loadVoltage > protOffMaxVoltage) {
        // We're off since more than n millis, but there's voltage at output. Don't allow turning
        // on in these conditions, and alert with leds.
        if (!protIsOutputPowered) {
            sup->telemetryAndPersistent(TELEMETRY_OUTPUT_IS_POWERED, TELEMETRY_PROT_VALUE, loadVoltage);
            sup->logger->error("Protection: Output has voltage while load is not turned on: {} mV", loadVoltage);
            protIsOutputPowered = true;
        }
    }
    if (!isLoadOn && ts - loadOffTs > 5 && loadVoltage <= protOffMaxVoltage) {
        // We're off and voltage at output has disappeared, just reset state
        // reset protIsOutputPowered
        if (protIsOutputPowered) {
            protIsOutputPowered = false;
            sup->telemetryAndPersistent(TELEMETRY_OUTPUT_IS_NOT_POWERED, TELEMETRY_PROT_VALUE, loadVoltage);
            sup->logger->info("Protection: Output has no voltage while load is not turned on: {} mV", loadVoltage);
        }
    }

    // If we're on and current is too high, turn off
    if (isLoadOn && loadCurrent > protOnMaxCurrent) {
        if (!isTurnOffInitiated) {
            isTurnOffInitiated = true;
            sup->logger->error("Protection: Overcurrent while load is on: {} mA, turning load off", loadCurrent);
            if (protIsEnabled) {
                sup->protDoLoadOff();
            }
            sup->telemetryAndPersistent(TELEMETRY_LOG_OVERCURRENT, TELEMETRY_PROT_VALUE, loadCurrent);
            sup->setPersistentState(PERSISTENT_STATE_OVERCURRENT, sup->getPersistentState(PERSISTENT_STATE_OVERCURRENT) + 1);
            // forced telemetry upload
            sup->initiateTelemetryUpload();
            goto endOfConditions; // we can't use the previous measurements anymore, because we just turned the load off
        }
// - When on load and overcurrent, stop load and increment a counter for this condition. If the counter becomes > 5, don't run the load anymore. If a load terminates without overcurrent, reset the counter.
// TODO
    }

    if (isLoadOn && ts - loadOnTs > 5 && loadVoltage < protOnMinVoltage) {
        // We have low voltage at output when on - the fuse may have blown. Just log.
        if (!protIsLowOutputVoltage) {
            sup->telemetryAndPersistent(TELEMETRY_LOW_OUTPUT_VOLTAGE, TELEMETRY_PROT_VALUE, loadVoltage);
            protIsLowOutputVoltage = true;
            sup->logger->error("Protection: Low output voltage while load is on: {} mV", loadVoltage);
        }
    }
    if (isLoadOn && ts - loadOnTs > 5 && loadVoltage >= protOnMinVoltage) {
        // After having had low output voltage, we have correct output voltage. Just log.
        if (protIsLowOutputVoltage) {
            sup->telemetryAndPersistent(TELEMETRY_NORMAL_OUTPUT_VOLTAGE, TELEMETRY_PROT_VALUE, loadVoltage);
            protIsLowOutputVoltage = false;
            sup->logger->info("Protection: Normal output voltage while load is on: {} mV", loadVoltage);
        }
    }

    if (mainVoltage < protMinVoltageShutdown) {
        // If input voltage is really too low, shut down and don't wake up
        if (isLoadOn) {
            if (!isTurnOffInitiated) {
                isTurnOffInitiated = true;
                sup->logger->error("Protection: Main voltage is low: {} mV, turning load off, will shut down", mainVoltage);
                if (protIsEnabled) {
                    sup->protDoLoadOff();
                }
            }
        }
        if (!isShutdownInitiated) {
            isShutdownInitiated = true;
            sup->telemetryAndPersistent(TELEMETRY_SHUTDOWN_UNDERVOLTAGE, TELEMETRY_PROT_VALUE, mainVoltage);
            // forced telemetry upload
            sup->initiateTelemetryUpload();
            sup->logger->error("Protection: Main voltage is low: {} mV, shutting down with no wakeup", mainVoltage);
            if (protIsEnabled) {
                sup->protDoShutdownNoWakeup();
            }
            goto endOfConditions; // don't use the previous measurements
        }
    }
    if (mainVoltage < protMinVoltageToLoad) {
        // On or off, if input voltage is too low, turn load off if needed and note it (load won't be turned on)
        // undervoltage, turn off and don't turn on
        bool turnedOff = false;
        if (isLoadOn) {
            if (!isTurnOffInitiated) {
                isTurnOffInitiated = true;
                sup->logger->error("Protection: Main voltage is low: {} mV, turning load off", mainVoltage);
                if (protIsEnabled) {
                    sup->protDoLoadOff();
                }
                turnedOff = true;
            }
        }
        if (!protIsUndervoltage) {
            sup->telemetryAndPersistent(TELEMETRY_LOG_UNDERVOLTAGE, TELEMETRY_PROT_VALUE, mainVoltage);
            if (turnedOff) { // just to keep the order of telemetry this way
                sup->telemetryAndPersistent(TELEMETRY_TURN_OFF_UNDERVOLTAGE, TELEMETRY_PROT_VALUE, mainVoltage);
            }
            // forced telemetry upload
            sup->initiateTelemetryUpload();
            sup->logger->error("Protection: Main voltage is low: {} mV (load is off)", mainVoltage);
            protIsUndervoltage = true;
        }
        if (turnedOff) {
            goto endOfConditions; // don't use the previous measurements, we just turned the load off
        }
    }
    if (mainVoltage >= protMinVoltageToLoad) {
        // Reset the condition, mainly for testing
        protIsUndervoltage = false;
    }


    if (caseTemp != 999999 && caseTemp > protMaxTempShutdown) {
        // If overtemperature for shutdown, turn off and shut down
        if (isLoadOn) {
            if (!isTurnOffInitiated) {
                isTurnOffInitiated = true;
                sup->telemetryAndPersistent(TELEMETRY_TURN_OFF_OVERTEMP, TELEMETRY_PROT_VALUE, caseTemp);
                sup->logger->error("Protection: Overtemperature: {} m°C, turning the load off and shutting down", caseTemp);
                if (protIsEnabled) {
                    sup->protDoLoadOff();
                }
            }
        }
        if (!isShutdownInitiated) {
            isShutdownInitiated = true;
            sup->telemetryAndPersistent(TELEMETRY_SHUTDOWN_OVERTEMP, TELEMETRY_PROT_VALUE, caseTemp);
            // forced telemetry upload
            sup->initiateTelemetryUpload();
            sup->logger->error("Protection: Overtemperature: {} m°C, shutting down", caseTemp);
            if (!isShutdownInitiated) {
                if (protIsEnabled) {
                    sup->protDoShutdown();
                }
            }
            goto endOfConditions; // don't use the previous measurements, we just turned the load off
        }
    }
    // If on, on overtemperature, turn off
    if (isLoadOn && caseTemp != 999999 && caseTemp > protMaxTempToLoad) {
        if (!protIsOvertemp) {
            protIsOvertemp = true;
            if (!isTurnOffInitiated) {
                isTurnOffInitiated = true;
                sup->logger->error("Protection: Overtemperature: {} m°C, turning the load off", caseTemp);
                if (protIsEnabled) {
                    sup->protDoLoadOff();
                }
                sup->telemetryAndPersistent(TELEMETRY_TURN_OFF_OVERTEMP, TELEMETRY_PROT_VALUE, caseTemp);
            } else {
                sup->telemetryAndPersistent(TELEMETRY_OVERTEMP, TELEMETRY_PROT_VALUE, caseTemp);
            }
            // forced telemetry upload
            sup->initiateTelemetryUpload();
            goto endOfConditions; // don't use the previous measurements, we just turned the load off
        }
    } else if (!isLoadOn && caseTemp != 999999 && caseTemp > protMaxTempToLoad) {
        if (!protIsOvertemp) {
            protIsOvertemp = true;
            sup->telemetryAndPersistent(TELEMETRY_OVERTEMP, TELEMETRY_PROT_VALUE, caseTemp);
        }
    } else if (caseTemp != 999999 && caseTemp <= protMaxTempToLoad) {
        if (protIsOvertemp) {
            protIsOvertemp = false;
            sup->telemetryAndPersistent(TELEMETRY_NOT_OVERTEMP, TELEMETRY_PROT_VALUE, caseTemp);
        }
    }
    if (caseTemp == 999999) {
        // we have temp reading error. Write this down
        if (!protIsTempError) {
            sup->telemetryAndPersistent(TELEMETRY_TEMP_ERROR);
            protIsTempError = true;
        }
    } else {
        if (protIsTempError) {
            sup->telemetryAndPersistent(TELEMETRY_TEMP_NOT_ERROR, TELEMETRY_PROT_VALUE, caseTemp);
            protIsTempError = false;
        }
    }

    // if protection is not enabled, reset turn off and shutdown flags if measurements are OK, used for testing

    if (!protIsCrowbar && !(!isLoadOn && ts - loadOffTs > 5 && loadCurrent > protOffMaxCurrent)) {
        protIsCrowbar = false;
    }
    if (!protIsEnabled && isShutdownInitiated &&
            !(caseTemp != 999999 && caseTemp > protMaxTempShutdown)
            && !(mainVoltage < protMinVoltageShutdown)) {
        isShutdownInitiated = false;
    }
    if (!protIsEnabled && isTurnOffInitiated &&
            !(isLoadOn && loadCurrent > protOnMaxCurrent)
            && !(mainVoltage < protMinVoltageShutdown)
            && !(mainVoltage < protMinVoltageToLoad)
            && !(caseTemp != 999999 && caseTemp > protMaxTempShutdown)
            && !(isLoadOn && caseTemp != 999999 && caseTemp > protMaxTempToLoad)) {
        isTurnOffInitiated = false;
    }


    endOfConditions:

    // If sensor measurements are in load mode, we keep track of the voltage and current profile
    if (isLoadOn || (!isLoadOn && ts - loadOffTs < 5000)) { // we'll gather profile until 5s second after turning off
        if (isFirstProfile) {
            lastProfileTs = loadOnTs;
            lastMeasureTs = ts;
            lastLoadCurrent = 0;
            lastLoadVoltage = 0;
            currentOrVoltageChanged = true;
            currentProfile.clear();
            varintAdd(currentProfile, 1, false); // version of the profile
            varintAdd(currentProfile, currentHyst, false);
            varintAdd(currentProfile, voltageHyst, false);
        }

        if (isFirstProfile || currentOrVoltageChanged) {
            // The load profile is a suite of entries of the following form:
            // - Timestamp difference with last entry (time since turn-on for the first entry)
            // - Timestamp difference with last measure timestamp (normally equal to period between two measures)
            // - Current difference with previous entry, in 10 mA units
            // - Voltage difference with previous entry, in 20 mV units
            // Data is encoded in varint, timestamps are positive, measures are signed (svarint)

            // guard from entering too many values in the profile
            int maxEntries = 100;
            if (currentProfile.size() < maxEntries * 4) {
                varintAdd(currentProfile, ts - lastProfileTs, false);
                varintAdd(currentProfile, ts - lastMeasureTs, false);
                varintAdd(currentProfile, (loadCurrent + 5) / 10 - (lastLoadCurrent + 5) / 10, true);
                varintAdd(currentProfile, (loadVoltage + 10) / 20 - (lastLoadVoltage + 10) / 20, true);
                lastProfileTs = ts;
                lastLoadCurrent = loadCurrent;
                lastLoadVoltage = loadVoltage;
            } else if (currentProfile.size() == maxEntries * 4) {
                // add one last entry with 999999
                varintAdd(currentProfile, ts - lastProfileTs, false);
                varintAdd(currentProfile, ts - lastMeasureTs, false);
                varintAdd(currentProfile, 999999, true);
                varintAdd(currentProfile, 999999, true);
            }
            // else we won't add another entry, the last added one is 999999
        }
        lastMeasureTs = ts;
        isFirstProfile = false;
    } else {
        if (isFirstProfile == false) { // first time we enter here after terminating the profile gathering
            // add one more entry with current values, without thresholds
            varintAdd(currentProfile, ts - lastProfileTs, false);
            varintAdd(currentProfile, ts - lastMeasureTs, false);
            varintAdd(currentProfile, (loadCurrent + 5) / 10 - (lastLoadCurrent + 5) / 10, true);
            varintAdd(currentProfile, (loadVoltage + 10) / 20 - (lastLoadVoltage + 10) / 20, true);

            // log the whole profile
            String profile;
            int32_t current = 0, voltage = 0;
            auto ptr = currentProfile.begin();
            uint32_t version = varintDecode(currentProfile, ptr, false);
            uint16_t currentHyst = varintDecode(currentProfile, ptr, false);
            uint16_t voltageHyst = varintDecode(currentProfile, ptr, false);
            profile.concat("version "); profile.concat(version);
            profile.concat(" currentHyst "); profile.concat(currentHyst);
            profile.concat(" voltageHyst "); profile.concat(voltageHyst);
            profile.concat("\n");
            while (ptr != currentProfile.end()) {
                ts += varintDecode(currentProfile, ptr, false);
                unsigned long lastTsDelta = varintDecode(currentProfile, ptr, false);
                int c = varintDecode(currentProfile, ptr, true);
                int v = varintDecode(currentProfile, ptr, true);
                current += (c == 999999 ? 0 : c * 10);
                voltage += (v == 999999 ? 0 : v * 20);
                profile += "{\"ts\":" + String(ts) + ",\"tsd\":" + String(lastTsDelta)
                        + ",\"i\":" + String(current) + ",\"v\":" + String(voltage) + "}\n";
            }
            sup->logger->debug("Load profile: {}", profile.c_str());
            isFirstProfile = true;
        }
    }

    // move forward loadOnTs and loadOffTs, so that they don't get too much behind current time
    // (and fail to work because of wraparound of millis())
    // keep them at 1h ago
    if (ts - loadOnTs > 60 * 1000 * 1000) {
        loadOnTs = ts - 60 * 1000 * 1000;
    }
    if (ts - loadOffTs > 60 * 1000 * 1000) {
        loadOffTs = ts - 60 * 1000 * 1000;
    }

}

bool Sensors::protIsDoNotLoad()
{
    return protIsOutputPowered || protIsUndervoltage || protIsOvertemp;
}

void Sensors::readLoadProfile(String *profileStr)
{
    // first convert the profile into a string
    Util::base64Encode(profileStr, (char *)currentProfile.data(), currentProfile.size());
}

void Sensors::spotMeasure()
{
return;
    unsigned long ts = millis();
    bool isHighFreq = (isLoadOn || (!isLoadOn && ts - loadOffTs <= 5000));
    doSensorMeasurements(isHighFreq);
}

void Sensors::readSensors(int *mainVoltage, int *mainCurrent, int *loadVoltage, int *loadCurrent,
    int *caseTemp, int *envTemp, int *envHumidity, int *sensorBatteryPct, int *sensorRssi, int *rssi)
{
    *mainVoltage = this->mainVoltage;
    *mainCurrent = this->mainCurrent;
    *loadVoltage = this->loadVoltage;
    *loadCurrent = this->loadCurrent;
    *caseTemp = this->caseTemp;

#ifdef USE_AM2320
    sup->am2320->getMeasures(envTemp, envHumidity);
#endif
#ifdef USE_BLE
    if (!bleSensorAddress.isEmpty()) {
        BleSensor *sensor = sup->ble->getSensor(bleSensorAddress.c_str());
        if (sensor != nullptr) {
            BleSensorMeasures measures;
            sensor->getMeasures(&measures);
            *envTemp = measures.temp;
            *envHumidity = measures.humidity;
            *sensorBatteryPct = measures.sensorBatteryPct;
            *sensorRssi = measures.sensorRssi;
        }
    }
#endif
#ifdef USE_ESP32I2C
    sup->esp32i2c->getMeasures(envTemp, envHumidity, sensorBatteryPct, sensorRssi);
#endif
    *rssi = sup->sim7000->getRssi();
}


//  .d8888b.                                                              888          
// d88P  Y88b                                                             888          
// 888    888                                                             888          
// 888         .d88b.  88888b.d88b.  88888b.d88b.   8888b.  88888b.   .d88888 .d8888b  
// 888        d88""88b 888 "888 "88b 888 "888 "88b     "88b 888 "88b d88" 888 88K      
// 888    888 888  888 888  888  888 888  888  888 .d888888 888  888 888  888 "Y8888b. 
// Y88b  d88P Y88..88P 888  888  888 888  888  888 888  888 888  888 Y88b 888      X88 
//  "Y8888P"   "Y88P"  888  888  888 888  888  888 "Y888888 888  888  "Y88888  88888P' 


void Sensors::initCommands(ServiceCommands *cmd)
{
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("prot.enabled", true)
        .cmdOn("prot.enable")
        .helpOn("--> Enable sensor-based protection, except crowbar protection.")
        .cmdOff("prot.disable")
        .helpOff("--> Disable sensor-based protection, except crowbar protection")
        .cmd("enable")
        .ptr(&protIsEnabled)
    );
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("prot.crowbarEnabled", true)
        .cmdOn("prot.crowbarEnable")
        .helpOn("--> Enable sensor-based crowbar protection.")
        .cmdOff("prot.crowbarDisable")
        .helpOff("--> Disable sensor-based crowbar protection")
        .cmd("crowbarEnable")
        .ptr(&protCrowbarIsEnabled)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.highFrequencyMillis", true)
        .cmd("prot.highFrequencyMillis")
        .help("--> Period of sensor measuring for high frequency period, during load on.")
        .vMin(1)
        .ptr(&highFrequencyMillis)
        .setFn([this](int val, bool isLoading, String *msg) {
            highFrequencyMillis = val;
            // if (!(!isLoading && (isLoadOn || (!isLoadOn && millis() - loadOffTs <= 5000)))) {
            //     timer.cancelTimeout();
            //     timer.setTimeout(highFrequencyMillis);
            // }
            *msg = "Set highFrequencyMillis to "; msg->concat(highFrequencyMillis);
            return true;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.lowFrequencyMillis", true)
        .cmd("prot.lowFrequencyMillis")
        .help("--> Period of sensor measuring for low frequency period, outside of load on.")
        .vMin(1)
        .ptr(&lowFrequencyMillis)
        .setFn([this](int val, bool isLoading, String *msg) {
            lowFrequencyMillis = val;
            // if (!isLoading && (isLoadOn || (!isLoadOn && millis() - loadOffTs <= 5000))) {
            //     timer.cancelTimeout();
            //     timer.setTimeout(lowFrequencyMillis);
            // }
            *msg = "Set lowFrequencyMillis to "; msg->concat(lowFrequencyMillis);
            return true;
        })
    );

    cmd->registerFloatData(
        ServiceCommands::FloatDataBuilder("prot.currentProfileHysteresisPct", true)
        .cmd("prot.currentProfileHysteresisPct")
        .help("--> Pct. of change in current needed to store a new value in the load profile.")
        .vMin(0.1)
        .vMax(100)
        .ptr(&hystCurrentPct)
    );
    cmd->registerFloatData(
        ServiceCommands::FloatDataBuilder("prot.voltageProfileHysteresisPct", true)
        .cmd("prot.voltageProfileHysteresisPct")
        .help("--> Pct. of change in voltage needed to store a new value in the load profile.")
        .vMin(0.1)
        .vMax(100)
        .ptr(&hystVoltagePct)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.offMaxCurrent", true)
        .cmd("prot.offMaxCurrent")
        .help("--> Maximum load current allowed when turned off, in milliamps. Ideally would be 0. If exceeded, we'll blow the fuse.")
        .vMin(0)
        .ptr(&protOffMaxCurrent)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.offMaxVoltage", true)
        .cmd("prot.offMaxVoltage")
        .help("--> Maximum voltage allowed when turned off, in milliamps. Ideally would be 0. If exceeded, we won't turn on the load.")
        .vMin(0)
        .ptr(&protOffMaxVoltage)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.onMaxCurrent", true)
        .cmd("prot.onMaxCurrent")
        .help("--> Maximum load current allowed when turned on, in milliamps. If exceeded, we'll turn the load off.")
        .vMin(0)
        .ptr(&protOnMaxCurrent)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.onMinVoltage", true)
        .cmd("prot.onMinVoltage")
        .help("--> Minimum voltage expected on load when turned on, in millivolts. If lower, we'll assume the fuse has blown, the load is not functioning.")
        .vMin(0)
        .ptr(&protOnMinVoltage)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.minVoltageToLoad", true)
        .cmd("prot.minVoltageToLoad")
        .help("--> Minimum battery voltage for turning on the load. If lower, we won't turn on the load.")
        .vMin(0)
        .ptr(&protMinVoltageToLoad)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.minVoltageShutdown", true)
        .cmd("prot.minVoltageShutdown")
        .help("--> Minimum battery voltage for operating. If lower, we'll shut down without wakeup.")
        .vMin(0)
        .ptr(&protMinVoltageShutdown)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.maxTempToLoad", true)
        .cmd("prot.maxTempToLoad")
        .help("--> Maximum temperature for turning on the load. If higher, we'll turn the load off, and will not turn it on.")
        .vMin(0)
        .ptr(&protMaxTempToLoad)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.maxTempShutdown", true)
        .cmd("prot.maxTempShutdown")
        .help("--> Maximum temperature for operating. If higher, we'll shut down, with programmed wakeup.")
        .vMin(0)
        .ptr(&protMaxTempShutdown)
    );
    
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.status.isCrowbar", true)
        .cmd("prot.status.isCrowbar")
        .help("--> If true, we have activated the crowbar.")
        .isPersistent(false)
        .getFn([this]() { return protIsCrowbar; })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.status.isOutputPowered", true)
        .cmd("prot.status.isOutputPowered")
        .help("--> If true, output has a voltage but we haven't turned the load on.")
        .isPersistent(false)
        .getFn([this]() { return protIsOutputPowered; })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.status.isUndervoltage", true)
        .cmd("prot.status.isUndervoltage")
        .help("--> If true, main voltage is too low, we won't turn the load on.")
        .isPersistent(false)
        .getFn([this]() { return protIsUndervoltage; })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.status.isOvertemp", true)
        .cmd("prot.status.isOvertemp")
        .help("--> If true, the temperature in the case is too high, we won't turn the load on.")
        .isPersistent(false)
        .getFn([this]() { return protIsOvertemp; })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.status.isLowOutputVoltage", true)
        .cmd("prot.status.isLowOutputVoltage")
        .help("--> If true, output voltage is low when the load is on, the fuse may have blown.")
        .isPersistent(false)
        .getFn([this]() { return protIsLowOutputVoltage; })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.measurements.mainVoltage", true)
        .cmd("prot.measurements.mainVoltage")
        .help("--> Main voltage.")
        .isPersistent(false)
        .getFn([this]() { return mainVoltage; })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.measurements.mainCurrent", true)
        .cmd("prot.measurements.mainCurrent")
        .help("--> Main current.")
        .isPersistent(false)
        .getFn([this]() { return mainCurrent; })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.measurements.loadVoltage", true)
        .cmd("prot.measurements.loadVoltage")
        .help("--> Load voltage.")
        .isPersistent(false)
        .getFn([this]() { return loadVoltage; })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.measurements.loadCurrent", true)
        .cmd("prot.measurements.loadCurrent")
        .help("--> Load current.")
        .isPersistent(false)
        .getFn([this]() { return loadCurrent; })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.measurements.caseTemp", true)
        .cmd("prot.measurements.caseTemp")
        .help("--> Case temperature.")
        .isPersistent(false)
        .getFn([this]() { return caseTemp; })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.measurements.envTemp", true)
        .cmd("prot.measurements.envTemp")
        .help("--> Ambient temperature.")
        .isPersistent(false)
        .getFn([this]() {
#ifdef USE_AM2320
            int envTemp, envHumidity;
            sup->am2320->getMeasures(&envTemp, &envHumidity);
#endif
#ifdef USE_ESP32I2C
            int envTemp, envHumidity, sensorBatteryPct, sensorRssi;
            sup->esp32i2c->getMeasures(&envTemp, &envHumidity, &sensorBatteryPct, &sensorRssi);
#endif
            return envTemp;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("prot.measurements.envHumidity", true)
        .cmd("prot.measurement.envHumidity")
        .help("--> Ambient humidity.")
        .isPersistent(false)
        .getFn([this]() {
#ifdef USE_AM2320
            int envTemp, envHumidity;
            sup->am2320->getMeasures(&envTemp, &envHumidity);
#endif
#ifdef USE_ESP32I2C
            int envTemp, envHumidity, sensorBatteryPct, sensorRssi;
            sup->esp32i2c->getMeasures(&envTemp, &envHumidity, &sensorBatteryPct, &sensorRssi);
#endif
            return envHumidity;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("sensor.ble.addr", true)
        .cmd("sensor.ble.addr")
        .help("--> Address of a BLE sensor for temperature/humidity (must already be configured in ble service)")
        .ptr(&bleSensorAddress)
    );
}

#endif
