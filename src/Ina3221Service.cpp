#include <CompilationOpts.h>

#ifdef USE_INA3221

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "SystemService.h"
#include "LogMgr.h"
#include "Ina3221Service.h"

void Ina3221Service::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr, SystemService *systemService)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("ina3221");

    isEnabled = false;
    i2cPort = I2C_NUM_1;
    i2cAddress = 0x40;
    channelEnabled[0] = true;
    channelEnabled[1] = true;
    channelEnabled[2] = true;
    shuntResistance[0] = 100;
    shuntResistance[1] = 100;
    shuntResistance[2] = 100;
    currentReg = 128; // a non-existent reg, uint_8
    lastError.clear();
    loggingIntervalMillis = 1000;
    timer.init(eventLoop, [this](UEventLoopTimer *t) {
        if (!isEnabled) {
            return;
        }
        for (int i = 1; i <= 3; i++) {
            if (channelEnabled[i - 1]) {
                int voltage, current;
                bool rc = readChannelData(i, &voltage, &current);
                if (rc) {
                    logger->trace("Channel {}: {} mV, {} mA", i, voltage, current);
                } else {
                    logger->error("Channel {}: error reading i2c", i);
                }
            }
        }
    });

    ServiceCommands *cmd = commandMgr->getServiceCommands("ina3221");
    initCommands(cmd);

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
    this->cmd = nullptr;

    if (!isEnabled) {
        i2cPort = (i2c_port_t)-1;
        lastError = "Not enabled";
    } else {
        rc = systemService->getIdf(i2cPort);
        if (!rc) {
            logger->error("Error getting get the i2c driver, the service will not work");
            i2cPort = (i2c_port_t)-1;
            lastError = "Couldn't get driver";
        }

        // init hardware
        bool hwOk = true;
        if (i2cPort != (i2c_port_t)-1 && !identify()) {
            logger->error("Could not identify INA3221 at address {} in i2c port {}", i2cAddress, (int)i2cPort);
            i2cPort = (i2c_port_t)-1;
            hwOk = false;
        }
        if (hwOk) { // initialize
            bool ok = setupDeviceNormal();
            if (!ok) {
                lastError = "I2C error initializing device";
                i2cPort = (i2c_port_t)-1;
                hwOk = false;
            }
        }
        if (hwOk && loggingIntervalMillis >= 10) {
            timer.setInterval(loggingIntervalMillis);
        }
    }
}

bool Ina3221Service::setupDeviceNormal()
{
    if (!isEnabled) {
        return false;
    }
    int chEn =
        (channelEnabled[0] ? 0b100 : 0)
        | (channelEnabled[1] ? 0b010 : 0)
        | (channelEnabled[2] ? 0b001 : 0);

    bool ok =
        writeRegister(0x00, 0b10000000) // reset
        && writeRegister(0x00,
            (0 << 15) // not reset
            | (chEn << 12) // enable chosen channels
            | (0b001 << 9) // averaging 4
            | (0b100 << 6) // bus voltage time 1.1ms
            | (0b100 << 3) // shunt voltage time 1.1ms
            | (0b111 << 0) // mode shunt and bus continuous
        );
    return ok;
}

bool Ina3221Service::setupDeviceCheckOvercurrent(int channel)
{
    if (!isEnabled) {
        return false;
    }
    int chEn =
        (channel == 1 ? 0b100 : 0)
        | (channel == 2 ? 0b010 : 0)
        | (channel == 3 ? 0b001 : 0);

    bool ok = writeRegister(0x00,
            (0 << 15) // not reset
            | (chEn << 12) // enable chosen channels
            | (0b000 << 9) // averaging 1
            | (0b000 << 6) // bus voltage time 140us (but won't be used, as we'll set mode shunt only)
            | (0b010 << 3) // shunt voltage time 333us
            | (0b101 << 0) // mode shunt continuous
        );
    return ok;
}

bool Ina3221Service::setModeOvercurrent(int channel)
{
    return setupDeviceCheckOvercurrent(channel);
}

bool Ina3221Service::setModeNormal()
{
    return setupDeviceNormal();
}

bool Ina3221Service::readRegister(uint8_t reg, uint16_t *value)
{
    if (cmd != nullptr) {
        i2c_cmd_link_delete(cmd);
        cmd = nullptr;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    if (currentReg != reg) { // set register pointer
        i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x0 /* write */, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_start(cmd);
    }
    i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x1 /* read */, true);
    uint8_t data[2];
    i2c_master_read(cmd, data, 2, i2c_ack_type_t::I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2cPort, cmd, 200 / portTICK_PERIOD_MS);
    // delete cmd lazily, above

    if (err == ESP_OK) {
        *value = data[1] + (data[0] << 8);
        lastError.clear();
        currentReg = reg;
        return true;
    } else {
        lastError = "Error reading i2c";
        currentReg = 128; // unknown reg
        return false;
    }
}

bool Ina3221Service::writeRegister(uint8_t reg, uint16_t value)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    if (currentReg != reg) { // set register pointer
        i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x0 /* write */, true);
        i2c_master_write_byte(cmd, reg, true);
        i2c_master_start(cmd);
    }
    i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x0 /* write */, true);
    i2c_master_write_byte(cmd, (uint8_t)(value & 0xFF), true);
    i2c_master_write_byte(cmd, (uint8_t)(value >> 8), true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2cPort, cmd, 200 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (err == ESP_OK) {
        lastError.clear();
        currentReg = reg;
        return true;
    } else {
        lastError = "Error writing i2c";
        currentReg = 128; // unknown reg
        return false;
    }
}

bool Ina3221Service::identify()
{
    uint16_t manufacturerId;
    uint16_t dieId;
    bool ok = readRegister(0xFE, &manufacturerId) && readRegister(0xFF, &dieId);
    if (!ok) {
        lastError = "Error reading i2c for identification";
        return false;
    }
    if (manufacturerId != 0x5449) {
        lastError = "Manufacturer ID is not 0x5449 but ";
        lastError.concat(manufacturerId);
        return false;
    }
    if (dieId != 0x3220) {
        lastError = "Die ID is not 0x3220 but ";
        lastError.concat(dieId);
        return false;
    }
    return true;
}

void Ina3221Service::initCommands(ServiceCommands *cmd)
{
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("enable", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .helpOn("--> Enable the sensor (requires reboot if not enabled upon startup)")
        .helpOff("--> Disable the sensor")
        .ptr(&isEnabled)
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("lastError", true)
        .cmd("lastError")
        .help("--> Last error, if any")
        .isPersistent(false)
        .getFn([this](String *msg) {
            if (lastError.isEmpty()) {
                *msg = "OK";
            } else {
                *msg = lastError;
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("loggingInterval", true)
        .cmd("loggingInterval")
        .help("--> Interval in milliseconds for automatic logging of measures, 0 for none, minimum 10")
        .ptr(&loggingIntervalMillis)
        .setFn([this](int val, bool isLoading, String *msg) {
            loggingIntervalMillis = val;
            if (!isLoading) {
                timer.cancelInterval();
                if (loggingIntervalMillis >= 10) {
                    timer.setInterval(loggingIntervalMillis);
                }
            }
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("reg", true)
        .cmd("reg")
        .help("reg <regNr> [ <newValue> ] --> Read or write a register")
        .isPersistent(false)
        .setFn([this](const String &val, bool isLoading, String *msg) -> bool {
            int regNr, value;
            int n = sscanf(val.c_str(), "%d %d", &regNr, &value);
            if (n == 1) {
                uint16_t data;
                bool rc = readRegister(regNr, &data);
                if (rc) {
                    *msg = data;
                } else {
                    *msg = "i2c error reading register "; msg->concat(regNr);
                }
            } else if (n == 2) {
                bool rc = writeRegister(regNr, value);
                if (rc) {
                    *msg = "Register "; msg->concat(regNr); msg->concat(" written");
                } else {
                    *msg = "i2c error writing register "; msg->concat(regNr);
                }
            } else {
                *msg = "Must specify register number, no action taken";
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("ch1R", true)
        .cmd("ch1R")
        .help("--> Channel 1 shunt resistance in milliohms")
        .vMin(1)
        .ptr(&shuntResistance[0])
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("ch2R", true)
        .cmd("ch2R")
        .help("--> Channel 2 shunt resistance in milliohms")
        .vMin(1)
        .ptr(&shuntResistance[1])
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("ch3R", true)
        .cmd("ch3R")
        .help("--> Channel 3 shunt resistance in milliohms")
        .vMin(1)
        .ptr(&shuntResistance[2])
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("ch1Enabled", true)
        .cmd("ch1Enabled")
        .help("ch1Enabled on|off --> Channel 1 enable measurements - requires reboot")
        .ptr(&channelEnabled[0])
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("ch2Enabled", true)
        .cmd("ch2Enabled")
        .help("ch2Enabled on|off --> Channel 2 enable measurements - requires reboot")
        .ptr(&channelEnabled[1])
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("ch3Enabled", true)
        .cmd("ch3Enabled")
        .help("ch3Enabled on|off --> Channel 3 enable measurements - requires reboot")
        .ptr(&channelEnabled[2])
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("ch1", true)
        .cmd("ch1")
        .help("--> Channel 1 shunt current in milliamperes, voltage in millivolts")
        .isPersistent(false)
        .getFn([this](String *val) {
            int voltage, current;
            bool rc = readChannelData(1, &voltage, &current);
            if (rc) {
                *val = String(voltage) + " mV, " + String(current) + " mA";
            } else {
                *val = "error";
            }
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("ch2", true)
        .cmd("ch2")
        .help("--> Channel 2 shunt current in milliamperes, voltage in millivolts")
        .isPersistent(false)
        .getFn([this](String *val) {
            int voltage, current;
            bool rc = readChannelData(2, &voltage, &current);
            if (rc) {
                *val = String(voltage) + " mV, " + String(current) + " mA";
            } else {
                *val = "error";
            }
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("ch3", true)
        .cmd("ch3")
        .help("--> Channel 3 shunt current in milliamperes, voltage in millivolts")
        .isPersistent(false)
        .getFn([this](String *val) {
            int voltage, current;
            bool rc = readChannelData(3, &voltage, &current);
            if (rc) {
                *val = String(voltage) + " mV, " + String(current) + " mA";
            } else {
                *val = "error";
            }
            return true;
        })
    );
}

bool Ina3221Service::readChannelData(int channel, int *voltage, int *current)
{
    if (!isEnabled || channel < 1 || channel > 3) {
        return false;
    }
    int r = shuntResistance[channel - 1];
    int v = 0, i = 0;
    uint16_t data;
    bool rc = readRegister((channel - 1) * 2 + 1, &data);
    if (rc) {
        i = ((int16_t)data / 8) * 40 / r;
        rc = readRegister((channel - 1) * 2 + 2, &data);
        if (rc) {
            // v = ((int16_t)data / 8) * 0.008 * 1000.0;
            v = ((int16_t)data / 8) * 8;
        }
    }
    if (rc) {
        *voltage = v;
        *current = i;
        return true;
    } else {
        return false;
    }
}

#endif

