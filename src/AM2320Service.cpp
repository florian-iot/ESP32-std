#include <CompilationOpts.h>

#ifdef USE_AM2320

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "SystemService.h"
#include "AM2320Service.h"

void AM2320Service::init(UEventLoop *eventLoop, CommandMgr *commandMgr, SystemService *systemService, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("am2320");

    ServiceCommands *cmd = commandMgr->getServiceCommands("am2320");
    initDfa();
    initCommands(cmd);

    // defaults
    isEnabled = false;
    i2cPort = I2C_NUM_0;
    i2cAddress = 0x5C;

    isMeasured = false;
    temp = 9999;
    humidity = 9999;

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
    if (isEnabled) {
        rc = systemService->getIdf(i2cPort);
        if (!rc) {
            logger->error("Cannot retrieve i2c port {} in IDF form from System service", (int)(i2cPort));
            isEnabled = false;
        }
    }
}

unsigned short AM2320Service::crc16(unsigned char *ptr, unsigned char len)
{
    unsigned short crc = 0xFFFF;
    unsigned char i;
    while (len--) {
        crc ^= *ptr++;
        for (i = 0; i < 8; i++) {
            if (crc & 0x01) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

bool AM2320Service::wakeUp() {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x0 /* write */, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2cPort, cmd, 20 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (err == ESP_OK);
}

bool AM2320Service::setReadRegs()
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x0 /* write */, true);
    i2c_master_write_byte(cmd, 0x03, true);
    i2c_master_write_byte(cmd, 0, true);
    i2c_master_write_byte(cmd, 4, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2cPort, cmd, 20 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (err == ESP_OK);
}

bool AM2320Service::readRegs() {
    uint8_t buf[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x1 /* read */, true);
    i2c_master_read(cmd, buf, 8, i2c_ack_type_t::I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2cPort, cmd, 20 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if (err != ESP_OK) {
        return false;
    }

    uint16_t humidityTmp = (buf[2] << 8) | buf[3];
    uint16_t tempTmp = (buf[4] << 8) | buf[5];
    uint16_t crc = (buf[7] << 8) | buf[6];
    uint16_t calculatedCrc = crc16(buf, 6);
    logger->debug("Received {}-{}, temp {}, humidity {}, calculated crc {} ({})",
        buf[0], buf[1], tempTmp, humidityTmp, calculatedCrc, (crc == calculatedCrc) ? "OK" : "Error");

    if (calculatedCrc == crc) {
        temp = (int16_t)tempTmp; // TODO check whether negative values are OK, or are they 1's complement?
        humidity = humidityTmp;
        isMeasured = true;
    }

    return true;
}

void AM2320Service::startMeasuring()
{
    dfa.queueInputForState(START_MEASURE, IDLE);
}

void AM2320Service::getMeasures(int *envTemp, int *envHumidity)
{
    *envTemp = temp * 100; // in milli°C
    *envHumidity = humidity * 100; // in milli-%
}

bool AM2320Service::isMeasuringDone()
{
    return isMeasured;
}

void AM2320Service::initDfa()
{
    dfa.init(eventLoop, logger /* or nullptr, or logger, to log */, STARTUP);

    dfa.onInput([this](Dfa *dfa, Dfa::State state, Dfa::Input input) {

        if (state.is(STARTUP)) {
            if (input.is(Dfa::Input::ENTER_STATE)) {
                return dfa->transitionTo(IDLE);
            }
        } else if (state.is(IDLE)) {
            if (input.is(START_MEASURE)) {
                isMeasured = false;
                if (isEnabled) {
                    bool rc = wakeUp();
                    logger->debug("Wakeup: {}", rc);
                }
                return dfa->transitionTo(MEASURE_1, 5);
            }
        } else if (state.is(MEASURE_1)) {
            if (input.is(Dfa::Input::TIMEOUT)) {
                if (isEnabled) {
                    bool rc = setReadRegs();
                    logger->debug("Set regs: {}", rc);
                }
                return dfa->transitionTo(MEASURE_2, 2);
            }
        } else if (state.is(MEASURE_2)) {
            if (input.is(Dfa::Input::TIMEOUT)) {
                if (isEnabled) {
                    bool rc = readRegs();
                    logger->debug("Read regs: {}", rc);
                }
                return dfa->transitionTo(IDLE);
            }
        }

        return dfa->transitionError();
    });
}

void AM2320Service::initCommands(ServiceCommands *cmd)
{
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("enable", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .helpOn("--> Enable the sensor (requires reboot if not enabled upon startup)")
        .helpOff("--> Disable the sensor")
        .ptr(&isEnabled)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("i2cPort", true)
        .cmd("i2cPort")
        .help("--> Set the I2C port, 0 or 1. Requires reboot.")
        .vMin(0)
        .vMax(1)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            i2cPort = (val == 0 ? I2C_NUM_0 : I2C_NUM_1);
            *msg = "I2C port set to "; *msg += (int)i2cPort; *msg += ", please reboot";
            return true;
        })
        .getFn([this]() {
            return (int)i2cPort;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("read", true)
        .cmdOn("read")
        .helpOn("--> Perform a reading")
        .isPersistent(false)
        .setFn([this](int value, bool isLoading, String *msg) {
            dfa.queueInputForState(START_MEASURE, IDLE);
            *msg = "Read initiated";
            return true;
        })
    );

}

#endif
