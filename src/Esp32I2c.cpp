#include <CompilationOpts.h>

#ifdef USE_ESP32I2C

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "Esp32I2c.h"
#include "LogMgr.h"
#include <driver/i2c.h>

void Esp32I2cService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, SystemService *system, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("esp32i2c");
    this->system = system;

    ServiceCommands *cmd = commandMgr->getServiceCommands("esp32i2c");
    initDfa();
    initCommands(cmd);

    // defaults

    isEnabled = false;
    i2cPort = I2C_NUM_0;
    i2cAddress = 0x1A;
    bleAddress = NimBLEAddress((uint64_t)0);
    measurementPeriodSecs = 600; // must be at least 15 sec, so that we have the time for the remote to sleep and wake up
    remoteBleScanDuration = 30000;
    nextMeasurementTime = 0; // will be set once we get a measurement

    isI2cRetrieved = false;
    clearMeasures();

    // load config
    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }
}

void Esp32I2cService::clearMeasures()
{
    temp = 99999;
    humidity = 99999;
    batteryPct = 99999;
    batteryVoltage = 99999;
    rssi = 99999;
    isMeasured = false;
}

bool Esp32I2cService::writeParams(int32_t scanDurationMillis, int32_t sleepDurationSecs, bool doNewScan)
{
    // Write
    // Byte 0: request code 0x81
    // * Byte 1-4: set duration between scans, in millis
    // * Byte 5-8: >0 for sleep mode, indicates auto-wakeup timeout in millis
    // * Byte 9: 1 for requesting new scan, else 0.

    uint8_t data[10];
    data[0] = 0x81;
    memcpy(&data[1], &scanDurationMillis, 4);
    memcpy(&data[5], &sleepDurationSecs, 4);
    data[9] = doNewScan ? 0x01 : 0x00;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x0 /* write */, true);
    i2c_master_write(cmd, data, 10, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2cPort, cmd, 20 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        logger->error("Error {} writing in writeParams(): {}", err, esp_err_to_name(err));
        return false;
    }

    return true;
}

bool Esp32I2cService::readVersion()
{
    // * Followed by a read:
    // * Byte 0: Version of response.
    // *      1: Version 1, follows what's described here. Next versions can only have longer lengths.
    // * 8 bytes (to keep some forward compatibility)
    // * Byte 0: Version of esp32 remote

    uint8_t data[9];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x1 /* read */, true);
    i2c_master_read(cmd, data, 9, i2c_ack_type_t::I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2cPort, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        logger->error("Error {} reading in readData(): {}", err, esp_err_to_name(err));
        return false;
    }
    if (data[0] != 1) {
        logger->error("Bad response version, {}, in readData(): {}", data[0], esp_err_to_name(err));
        return false;
    }
    remoteVersion = data[1];
    return true;
}

bool Esp32I2cService::writeInitData()
{
    return writeRequest();
}

bool Esp32I2cService::writeRequest()
{
    // Write
    // Byte 0: request code 0x01
    // Byte 1: type of device
    // Bytes 2 - 7: MAC address of BLE device
    // Only public MAC addresses are used

    uint8_t data[8];
    data[0] = 0x01;
    data[1] = BLE_DEVICE_NOT_SPECIFIED; // for the time being we're supposing that matching can happen from BLE scan data
    memcpy(&data[2], bleAddress.getNative(), 6);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x0 /* write */, true);
    i2c_master_write(cmd, data, 8, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2cPort, cmd, 20 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        logger->error("Error {} writing in writeConfigData(): {}", err, esp_err_to_name(err));
        return false;
    }
    return true;
}

bool Esp32I2cService::readMeasures()
{
    // First read: 1 byte
    //    byte 0: number of available measurements

    // Second read: N * 5 bytes, N == number of available measurements
    //    byte 0: type of measurement
    //    bytes 1 - 4: value, int16_t, lsb first

    uint8_t count = 0;
    do { // reading returns 0xFF if no data is available at the remote side
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x1 /* read */, true);
        i2c_master_read(cmd, &count, 1, i2c_ack_type_t::I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        esp_err_t err = i2c_master_cmd_begin(i2cPort, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (err != ESP_OK) {
            logger->error("Error {} reading number of measurements in readData(): {}", err, esp_err_to_name(err));
            return false;
        }
    } while (count == 0xFF);

    logger->debug("Received count of measures: {}", count);

    if (count == 0) {
        return true; // but we had no data to get
    }

    uint8_t data[count * 5];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x1 /* read */, true);
    i2c_master_read(cmd, data, count * 5, i2c_ack_type_t::I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2cPort, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        logger->error("Error {} reading {} measurements in readData(): {}", err, count, esp_err_to_name(err));
        return false;
    }

    bool isPartial = false;
    for (int i = 0; i < count; i++) {
        uint8_t type = data[i * 5];
        int32_t val = data[i * 5 + 1]
            + (data[i * 5 + 2] << 8) + (data[i * 5 + 3] << 16) + (data[i * 5 + 4] << 24);
        logger->debug("Received measure {} of {}, type: {}, value: {}", i, count, type, val);
        switch (type) {
            case 1: temp = val; break;
            case 2: humidity = val; break;
            case 3: batteryPct = val; break;
            case 4: batteryVoltage = val; break;
            case 5: rssi = val; break;
            case 0xFF: isPartial = true; break;
            default:
                logger->debug("Received unknown measurement type {}, value is {}, ignored", type, val);
                break;
        }
    }
    isMeasured = !isPartial; // if we got partial data, just throw away the whole and retry getting
    return true;
}

// 8888888b.  8888888888     d8888 
// 888  "Y88b 888           d88888 
// 888    888 888          d88P888 
// 888    888 8888888     d88P 888 
// 888    888 888        d88P  888 
// 888    888 888       d88P   888 
// 888  .d88P 888      d8888888888 
// 8888888P"  888     d88P     888 

void Esp32I2cService::initDfa()
{
    dfa.init(eventLoop, logger, INITIAL);
    dfa.onInput([this](Dfa *dfa, Dfa::State state, Dfa::Input input) {

        if (state.is(INITIAL)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                if (isEnabled) {
                    return dfa->transitionTo(CONFIGURING);
                } else {
                    return dfa->transitionTo(COMM_DISABLED);
                }
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(CONFIGURING)) {

            if (input.is(Dfa::Input::ENTER_STATE, Dfa::Input::TIMEOUT)) {
                if (!isI2cRetrieved) {
                    bool rc = system->getIdf(i2cPort);
                    if (!rc) {
                        logger->error("Cannot retrieve i2c port {} in IDF form from System service", (int)(i2cPort));
                        return dfa->transitionTo(COMM_DISABLED);
                    }
                    i2c_filter_enable(i2cPort, 7);
                    isI2cRetrieved = true;
                }
                bool rc = writeParams(remoteBleScanDuration, 0, false);
                if (!rc) {
                    dfa->setStateTimeout(1000); // retry after a while
                    return dfa->noTransition();
                }
                isEnabled = true;
                return dfa->transitionTo(READING_VERSION, 10);
            } else if (input.is(DISABLE)) {
                isEnabled = false;
                return dfa->transitionTo(COMM_DISABLED);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(READING_VERSION)) {

            if (input.is(Dfa::Input::TIMEOUT)) {
                bool rc = readVersion();
                if (!rc) {
                    return dfa->transitionTo(CONFIGURING);
                }
                logger->info("Remote esp32 version: {}", this->remoteVersion);
                return dfa->transitionTo(WRITE_REQUEST, 1); // needs timeout, a short one is ok
            } else if (input.is(DISABLE)) {
                isEnabled = false;
                return dfa->transitionTo(COMM_DISABLED);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(WRITE_REQUEST)) {

            if (input.is(Dfa::Input::TIMEOUT)) {
                bool rc = writeRequest();
                if (!rc) {
                    return dfa->transitionTo(CONFIGURING);
                }
                return dfa->transitionTo(READING, 10); // delay after write before reading
            } else if (input.is(DISABLE)) {
                isEnabled = false;
                return dfa->transitionTo(COMM_DISABLED);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(READING)) {

            if (input.is(Dfa::Input::TIMEOUT)) {
                bool rc = readMeasures();
                if (!rc) {
                    return dfa->transitionTo(CONFIGURING);
                }
                if (isMeasured) { // once we get measures, we try to send remote esp32 to deep sleep and go to IDLE
                    nextMeasurementTime = (long)millis() + measurementPeriodSecs * 1000;
                    // send remote to deep sleep
                    int32_t sleepDuration = (measurementPeriodSecs - 15); // it will wake up 15 seconds before our nextMeasurementTime
                    if (sleepDuration > 0) {
                        bool rc = writeParams(remoteBleScanDuration, sleepDuration, false);
                        if (!rc) {
                            logger->error("Error sending remote esp32 to deep sleep, continuing");
                        } else {
                            logger->debug("Sent remote esp32 to deep sleep for {} millis", sleepDuration);
                        }
                    }
                    return dfa->transitionTo(IDLE);
                } else {
                    return dfa->transitionTo(WRITE_REQUEST, 1000); // read again, until measurement is done
                }
            } else if (input.is(DISABLE)) {
                isEnabled = false;
                return dfa->transitionTo(COMM_DISABLED);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(IDLE)) {

            if (input.is(Dfa::Input::ENTER_STATE, Dfa::Input::TIMEOUT)) {
                long timeLeft = nextMeasurementTime - (long)millis();
                if (timeLeft > 10) { // stay in IDLE if still need to wait (more than 10 millis) 
                    dfa->setStateTimeout(timeLeft);
                    return dfa->noTransition();
                } else { // reacquire
                    clearMeasures();
                    return dfa->transitionTo(WRITE_REQUEST, 1); // needs timeout, a short one is ok here
                }
            } else if (input.is(DISABLE)) {
                isEnabled = false;
                return dfa->transitionTo(COMM_DISABLED);
            } else if (input.is(REACQUIRE)) {
                clearMeasures();
                return dfa->transitionTo(WRITE_REQUEST, 1); // needs timeout, a short one is ok here
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(COMM_DISABLED)) {

            if (input.is(ENABLE)) {
                return dfa->transitionTo(CONFIGURING);
            } else {
                return dfa->transitionError();
            }
            
        } else {
            return dfa->transitionError();
        }

    });

}


//  .d8888b.                                                              888          
// d88P  Y88b                                                             888          
// 888    888                                                             888          
// 888         .d88b.  88888b.d88b.  88888b.d88b.   8888b.  88888b.   .d88888 .d8888b  
// 888        d88""88b 888 "888 "88b 888 "888 "88b     "88b 888 "88b d88" 888 88K      
// 888    888 888  888 888  888  888 888  888  888 .d888888 888  888 888  888 "Y8888b. 
// Y88b  d88P Y88..88P 888  888  888 888  888  888 888  888 888  888 Y88b 888      X88 
//  "Y8888P"   "Y88P"  888  888  888 888  888  888 "Y888888 888  888  "Y88888  88888P' 


void Esp32I2cService::initCommands(ServiceCommands *cmd)
{
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("i2cPort", true)
        .cmd("i2cPort")
        .help("--> Set i2c port, 0 or 1, requires reboot")
        .vMin(0)
        .vMax(1)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            i2cPort = (i2c_port_t)val;
            *msg = "Set i2cPort to "; *msg += i2cPort; *msg += ", please reboot";
            return true;
        })
        .getFn([this]() {
            return i2cPort;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("i2cAddress", true)
        .cmd("i2cAddress")
        .help("--> Set i2c address, requires reboot")
        .vMin(1)
        .vMax(127)
        .ptr(&i2cAddress)
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("bleAddress", true)
        .cmd("bleAddress")
        .help("--> BLE mac address of the device to monitor, \"0\" to set to no address")
        .getFn([this](String *val) {
            val->clear();
            val->concat(std::string(bleAddress).c_str());
        })
        .setFn([this](const String &val, bool isLoading, String *msg) {
            if (val.compareTo("0") == 0) {
                bleAddress = NimBLEAddress((uint64_t)0);
                *msg = "Address set to 00:00:00:00:00:00";
            } else {
                NimBLEAddress addr(val.c_str());
                if ((uint64_t)addr == 0) {
                    *msg = "Invalid address!";
                } else {
                    bleAddress = addr;
                    *msg = "Address set to ";
                    msg->concat(std::string(bleAddress).c_str());
                }
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("measurementPeriodSecs", true)
        .cmd("measurementPeriodSecs")
        .help("--> Period between two measurements in seconds")
        .vMin(15)
        .ptr(&measurementPeriodSecs)
        .setFn([this](int val, bool isLoading, String *msg) {
            measurementPeriodSecs = val;
            *msg = String("Set measurementPeriodsSec to ") + val;
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("dfa.state", true)
        .cmd("dfa.state")
        .help("--> DFA state")
        .isPersistent(false)
        .getFn([this](String *val) {
            *val = dfa.stateName(dfa.getState());
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("enabled", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .helpOn("--> Enables the service")
        .helpOff("--> Disables the service")
        .ptr(&isEnabled)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (isLoading) {
                isEnabled = val;
                return true;
            } else {
                if (!isEnabled && val) {
                    if (dfa.getState() == COMM_DISABLED) {
                        dfa.handleInput(ENABLE);
                    }
                    *msg = "Enabling";
                } else if (isEnabled && !val) {
                    dfa.handleInput(DISABLE);
                    *msg = "Disabling";
                } else {
                    *msg = (val ? "Already enabled" : "Already disabled");
                }
            }
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("reacquire", true)
        .cmdOn("reacquire")
        .helpOn("--> Re-acquire measures from remote ESP32")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (dfa.getState() == IDLE) {
                dfa.handleInput(REACQUIRE);
                *msg = "Starting acquisition";
            } else {
                *msg = "Not in IDLE state, ignored";
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("sleep", true)
        .cmd("sleep")
        .help("--> sleep <seconds> Send direct command to remote esp32 to deep sleep")
        .isPersistent(false)
        .vMin(1)
        .vMax(3600)
        .setFn([this](int val, bool isLoading, String *msg) {
            if (dfa.getState() == IDLE) {
                bool rc = writeParams(remoteBleScanDuration, val, false);
                if (!rc) {
                    *msg = "Error writing params for sleep";
                } else {
                    *msg = String("Requesting sleep ") + val;
                }
            } else {
                *msg = "Not in IDLE state, ignored";
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("temperature", true)
        .cmd("temperature")
        .isPersistent(false)
        .ptr(&temp)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("humidity", true)
        .cmd("humidity")
        .isPersistent(false)
        .ptr(&humidity)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("batteryPct", true)
        .cmd("batteryPct")
        .isPersistent(false)
        .ptr(&batteryPct)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("batteryVoltage", true)
        .cmd("batteryVoltage")
        .isPersistent(false)
        .ptr(&batteryVoltage)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("rssi", true)
        .cmd("rssi")
        .isPersistent(false)
        .ptr(&rssi)
    );
}

void Esp32I2cService::getMeasures(int *envTemp, int *envHumidity, int *sensorBatteryPct, int *sensorRssi)
{
    *envTemp = temp;
    *envHumidity = humidity;
    *sensorBatteryPct = batteryPct;
    *sensorRssi = rssi;
}



#endif
