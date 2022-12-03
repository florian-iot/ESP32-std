#include "EEPROMDriver.h"
#include <driver/i2c.h>
#include <Wire.h>
#include <HardwareSerial.h>


EEPROMMasterDriver::EEPROMMasterDriver()
{
    lastErr = ESP_OK;
    deviceAddress = 0;
    eeWritePageSize = 64;
    initDone = false;
    isSynced = true;
    isDriverInstalled = false;
}

esp_err_t EEPROMMasterDriver::lastError()
{
    return lastErr;
}

const char *EEPROMMasterDriver::lastErrorStr()
{
    return esp_err_to_name(lastErr);
}

bool EEPROMMasterDriver::init(int sdaPin, int sclPin, int freqHz, int port, uint8_t deviceAddress) {

    this->port = (port == 0 ? I2C_NUM_0 : port == 1 ? I2C_NUM_1 : (i2c_port_t)-1);
    this->deviceAddress = deviceAddress;
    isSynced = true;
    lastErr = ESP_OK;

    if (sdaPin != -1 && sclPin != -1) {
        i2c_config_t conf;
        memset(&conf, 0, sizeof(conf));
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = (gpio_num_t)sdaPin;
        conf.scl_io_num = (gpio_num_t)sclPin;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = freqHz;
        lastErr = i2c_param_config(this->port, &conf);
        if (lastErr != ESP_OK) {
            return false;
        }
        lastErr = i2c_driver_install(this->port, conf.mode, 0, 0, 0);
        isDriverInstalled = (lastErr == ESP_OK);
    }
    initDone = (lastErr == ESP_OK);
    return initDone;
}

bool EEPROMMasterDriver::init(int port, uint8_t deviceAddress) {

    this->port = (port == 0 ? I2C_NUM_0 : port == 1 ? I2C_NUM_1 : (i2c_port_t)-1);
    this->deviceAddress = deviceAddress;
    isSynced = true;
    lastErr = ESP_OK;
    initDone = true;
    return initDone;
}

void EEPROMMasterDriver::terminate()
{
    if (!initDone) {
        return;
    }
    if (isDriverInstalled) {
        i2c_driver_delete(port);
    }
    initDone = false;
}

void EEPROMMasterDriver::setDeviceAddress(uint8_t deviceAddress)
{
    this->deviceAddress = deviceAddress;
}

void EEPROMMasterDriver::setWritePageSize(uint16_t pageSize)
{
    eeWritePageSize = pageSize;
}

bool EEPROMMasterDriver::writeByteDev(uint8_t deviceAddress, uint16_t eeAddress, uint8_t byte) {
    if (!initDone) {
        return false;
    }
    int32_t maxWaitTicks = 100 / portTICK_PERIOD_MS; // 100 ms + 1 tick
    int32_t tm = xTaskGetTickCount();
    do {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddress << 1) | 0x0 /* write */, true);
        i2c_master_write_byte(cmd, eeAddress >> 8, true);
        i2c_master_write_byte(cmd, eeAddress & 0xFF, true);
        i2c_master_write_byte(cmd, byte, true);
        i2c_master_stop(cmd);
        lastErr = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    } while (lastErr == ESP_FAIL && xTaskGetTickCount() - tm < maxWaitTicks);
    // repeat above as long as slave doesn't send ACK (it's still in a previous write cycle),
    // but for no longer than 100 ms
    isSynced = false;
    return (lastErr == ESP_OK);
}

bool EEPROMMasterDriver::syncDev(uint8_t deviceAddress) {
    if (!initDone) {
        return false;
    }
    if (isSynced) {
        return true;
    }
    int32_t maxWaitTicks = 100 / portTICK_PERIOD_MS; // 100 ms + 1 tick
    int32_t tm = xTaskGetTickCount();
    do {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddress << 1) | 0x0 /* write */, true);
        i2c_master_stop(cmd);
        lastErr = i2c_master_cmd_begin(port, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    } while (lastErr == ESP_FAIL && xTaskGetTickCount() - tm < maxWaitTicks);
    // repeat above as long as slave doesn't send ACK (it's still in a previous write cycle),
    // but for no longer than 100 ms
    if (lastErr == ESP_OK) {
        isSynced = true;
    }
    return (lastErr == ESP_OK);
}

bool EEPROMMasterDriver::writeDev(uint8_t deviceAddress, uint16_t eeAddress, uint8_t *data, size_t size) {
    if (!initDone) {
        return false;
    }
    const int32_t maxWaitTicks = 100 / portTICK_PERIOD_MS; // 100 ms + 1 tick

    uint16_t currentAddress = eeAddress;
    size_t remaining = size;
    uint8_t *ptr = data;
    size_t toWrite;

    do {
        uint16_t nextPageAddress = (currentAddress | (eeWritePageSize - 1)) + 1;
        toWrite = nextPageAddress - currentAddress;
        if (toWrite > remaining) {
            toWrite = remaining;
        }
        int32_t tm = xTaskGetTickCount();
        do {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (deviceAddress << 1) | 0x0 /* write */, true);
            i2c_master_write_byte(cmd, currentAddress >> 8, true);
            i2c_master_write_byte(cmd, currentAddress & 0xFF, true);
            i2c_master_write(cmd, ptr, toWrite, true);
            i2c_master_stop(cmd);
            lastErr = i2c_master_cmd_begin(port, cmd, 100 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
        } while (lastErr == ESP_FAIL && xTaskGetTickCount() - tm < maxWaitTicks);
        // repeat above as long as slave doesn't send ACK (it's still in a previous write cycle),
        // but for no longer than 100 ms
        if (lastErr == ESP_OK) {
            remaining -= toWrite;
            ptr += toWrite;
            currentAddress += toWrite;
        }
    } while (lastErr == ESP_OK && remaining > 0);
    isSynced = false;

    return (lastErr == ESP_OK);
}

bool EEPROMMasterDriver::readDev(uint8_t deviceAddress, uint16_t eeAddress, uint8_t *bytePtr, size_t size) {
    if (!initDone) {
        return false;
    }
    const int32_t maxWaitTicks = 100 / portTICK_PERIOD_MS; // 100 ms + 1 tick
    int32_t tm = xTaskGetTickCount();
    do {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddress << 1) | 0x0 /* write */, true);
        i2c_master_write_byte(cmd, eeAddress >> 8, true);
        i2c_master_write_byte(cmd, eeAddress & 0xFF, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (deviceAddress << 1) | 0x1 /* read */, true);
        i2c_master_read(cmd, bytePtr, size, i2c_ack_type_t::I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);
        lastErr = i2c_master_cmd_begin(port, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    } while (lastErr == ESP_FAIL && xTaskGetTickCount() - tm < maxWaitTicks);
    // repeat above as long as slave doesn't send ACK (it's still in a previous write cycle),
    // but for no longer than 100 ms
    isSynced = true;
    return (lastErr == ESP_OK);
}

