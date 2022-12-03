#include <CompilationOpts.h>

#ifdef USE_LM75A_TEMP

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LM75A_TempService.h"

bool LM75A::init(int i2cAddress, i2c_port_t i2cPort)
{
    this->i2cAddress = i2cAddress;
    this->i2cPort = i2cPort;
    return checkConnect();
}

void LM75A::shutdown()
{
    if (!checkConnect()) {
        return;
    }
    int r = regConfig | 0b1;
    if (writeReg8(REG_CONFIG, r)) {
        regConfig = r;
    }
}

void LM75A::wakeup()
{
    if (!checkConnect()) {
        return;
    }
    int r = regConfig & ~0b1;
	  if (writeReg8(REG_CONFIG, r)) {
        regConfig = r;
    }
}

bool LM75A::isShutdown()
{
    if (!checkConnect()) {
        return false;
    }
	  return (regConfig & 1) == 1;
}

int LM75A::getTempMilliC()
{
    if (!checkConnect()) {
        return 999999;
    }
    uint16_t result;
    if (!readReg16(REG_TEMP, &result)) {
        return 999999;
    }
    return result * 1000 / 256;
}

int LM75A::getHystTempMilliC()
{
    if (!checkConnect()) {
        return 999999;
    }
    uint16_t result;
    if (!readReg16(REG_THYST, &result)) {
        return 999999;
    }
    return result * 1000 / 256;
}

int LM75A::getFaultQueueValue()
{
    if (!checkConnect()) {
        return 1;
    }
	  int n = (regConfig & 0b00011000) >> 3;
    switch (n) {
      case 0: return 1;
      case 1: return 2;
      case 2: return 4;
      case 3: return 6;
      default: return 1;
    }
}

int LM75A::getOSTripTempMilliC()
{
    if (!checkConnect()) {
        return 999999;
    }
    uint16_t result;
    if (!readReg16(REG_TOS, &result)) {
        return 999999;
    }
    return result * 1000 / 256;
}

int LM75A::getOsPolarity()
{
    if (!checkConnect()) {
        return 0;
    }
  	return (regConfig & 0b100) >> 2;
}

int LM75A::getDeviceMode()
{
    if (!checkConnect()) {
        return 0;
    }
	  return (regConfig & 0b010) >> 1;
}

void LM75A::setHystTempMilliC(int tempMilliC)
{
    if (!checkConnect()) {
        return;
    }
  	writeReg16(REG_THYST, tempMilliC * 256 / 1000);
}

void LM75A::setOsTripTemperature(int tempMilliC)
{
    if (!checkConnect()) {
        return;
    }
  	writeReg16(REG_TOS, tempMilliC * 256 / 1000);
}

void LM75A::setFaultQueueValue(int value)
{
    if (!checkConnect()) {
        return;
    }
    int n;
    switch (value) {
      case 1: n = 0; break;
      case 2: n = 1; break;
      case 4: n = 2; break;
      case 6: n = 3; break;
      default: return;
    }
    int r = (regConfig & 0b11100111) | n;
  	if (writeReg8(REG_CONFIG, r)) {
        regConfig = r;
    }
}

void LM75A::setOsPolarity(int polarity)
{
    if (!checkConnect()) {
        return;
    }
    int r = (regConfig & 0b11111011) | (polarity == 0 ? 0 : 1);
  	if (writeReg8(REG_CONFIG, r)) {
        regConfig = r;
    }
}

void LM75A::setDeviceMode(int deviceMode)
{
    if (!checkConnect()) {
        return;
    }
    int r = (regConfig & 0b11111101) | (deviceMode == 0 ? 0 : 1);
	  if (writeReg8(REG_CONFIG, r)) {
        regConfig = r;
    }
}

bool LM75A::readConfig()
{
    uint8_t r;
  	bool rc = readReg8(REG_CONFIG, &r);
    if (rc) {
        regConfig = r;
    }
    return rc;
}

bool LM75A::checkProdId()
{
    uint8_t r;
    bool rc = readReg8(REG_PROD_ID, &r);
    if (rc) {
        return (r >> 4) == 0xA;
    } else {
        return false;
    }
}

int LM75A::readProdId()
{
    uint8_t r;
    bool rc = readReg8(REG_PROD_ID, &r);
    if (rc) {
        return r;
    } else {
        return -1;
    }
}

bool LM75A::isConnected()
{
    uint8_t r, rNew;
    bool rc = readReg8(REG_CONFIG, &r);
    rc = rc && writeReg8(REG_CONFIG, 0x0F);
    rc = rc && readReg8(REG_CONFIG, &rNew);
    rc = rc && writeReg8(REG_CONFIG, r);
    return rc && rNew == 0x0F;
}

bool LM75A::readRegister(uint8_t reg, uint8_t *values, int length)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x0 /* write */, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x1 /* read */, true);
    i2c_master_read(cmd, values, length, i2c_ack_type_t::I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2cPort, cmd, 200 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (err == ESP_OK);
}

bool LM75A::readReg8(uint8_t reg, uint8_t *value)
{
    return readRegister(reg, value, 1);
}

bool LM75A::readReg16(uint8_t reg, uint16_t *value)
{
    uint8_t values[2];
    bool rc = readRegister(reg, values, 2);
    if (rc) {
        *value = (values[0] << 8) | values[1];
    }
    return rc;
}

bool LM75A::writeRegister(uint8_t reg, uint8_t *values, int length)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2cAddress << 1) | 0x0 /* write */, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write(cmd, values, length, i2c_ack_type_t::I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(i2cPort, cmd, 200 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return (err == ESP_OK);
}

bool LM75A::writeReg8(uint8_t reg, uint8_t value)
{
    return writeRegister(reg, &value, 1);
}

bool LM75A::writeReg16(uint8_t reg, uint16_t value)
{
    uint8_t values[2];
    values[0] = value >> 8;
    values[1] = value & 0xFF;
    return writeRegister(reg, values, 2);
}




void LM75A_TempService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, SystemService *systemService, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("lm75a");

    ServiceCommands *cmd = commandMgr->getServiceCommands("lm75a");

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("enable", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .helpOn("--> Enable the sensor (requires reboot if it was not enabled from startup)")
        .helpOff("--> Disable the sensor")
        .ptr(&isEnabled)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("i2cAddress", true)
        .cmd("i2cAddress")
        .help("--> Set the I2C address as set by pins A0, A1, A2, between 72 (0x48) and 79 (0x4F). Requires reboot.")
        .vMin(0x48)
        .vMax(0x4f)
        .ptr(&i2cAddress)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            i2cAddress = val;
            *msg = "I2C address set to "; *msg += i2cAddress; *msg += ", please reboot";
            return true;
        })
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

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("tempOffset", true)
        .cmd("tempOffset")
        .help("--> Offset to add to temperature readings, in milli C")
        .ptr(&tempOffsetMilliC)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("read", true)
        .isPersistent(false)
        .cmd("read")
        .help("--> Read the current temperature")
        .isPersistent(false)
        .getFn([this]() {
            return this->readTempMilliC();
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("readReg8", true)
        .isPersistent(false)
        .cmd("readReg8")
        .help("readReg8 <n> --> Read the specified 8-bit register")
        .isPersistent(false)
        .setFn([this](int value, bool isLoading, String *msg) {
        uint8_t result;
        bool rc = lm75a.readReg8(value, &result);
        if (rc) {
            *msg = String((unsigned int)result);
        } else {
            *msg = "Error reading register";
        }
        return true;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("readReg16", true)
        .isPersistent(false)
        .cmd("readReg16")
        .help("readReg16 <n> --> Read the specified 16-bit register")
        .isPersistent(false)
        .setFn([this](int value, bool isLoading, String *msg) {
        uint16_t result;
        bool rc = lm75a.readReg16(value, &result);
        if (rc) {
            *msg = String(result);
        } else {
            *msg = "Error reading register";
        }
        return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("log", true)
        .cmd("log")
        .help("log on|off --> enables temperature logging to serial")
        .setFn([this](bool val, bool isLoading, String *msg) -> bool {
        if (val) {
            loggingTimer.setInterval(5000);
        } else {
            loggingTimer.cancelInterval();
        }
        return true;
        })
    );


    // defaults
    this->i2cAddress = 0x48;
    tempOffsetMilliC = 0;
    isEnabled = false;

    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    if (isEnabled) {
        rc = systemService->getIdf(i2cPort);
        if (!rc) {
            logger->error("Cannot retrieve i2c port {} in IDF form from System service", (int)(i2cPort));
        }
        if (rc) {
            rc = lm75a.init(i2cAddress, i2cPort);
            if (!rc) {
            logger->error("Initialization failed for i2cPort {}, i2cAddress {}",
                (int)i2cPort, i2cAddress);
            }
        }
    }
    if (rc) {
        loggingTimer.init(eventLoop, [this](UEventLoopTimer *timer) {
            if (isEnabled) {
                int temp = lm75a.getTempMilliC();
                logger->info("Temperature: {}", String(temp / 1000.0, 3).c_str());
            }
        });
    }

}

int LM75A_TempService::readTempMilliC()
{
    if (!isEnabled) {
        return 999999;
    }
    int t = lm75a.getTempMilliC();
    return t == 999999 ? 999999 : t + tempOffsetMilliC;
}


#endif
