#ifndef INCL_EEPROM_DRIVER
#define INCL_EEPROM_DRIVER

#include <driver/i2c.h>

class EEPROMMasterDriver {
    i2c_port_t port;
    uint8_t deviceAddress; // 0 for undefined, in which case it must be specified in each call
    bool initDone;
    bool isDriverInstalled;
    bool isSynced;
    esp_err_t lastErr;
    uint16_t eeSize; // EEPROM size in bytes, power of 2
    uint16_t eeWritePageSize; // page size for page writes, defaults to 64
public:
    EEPROMMasterDriver();

    esp_err_t lastError();
    const char *lastErrorStr();
    bool init(int sdaPin, int sclPin, int freqHz, int port, uint8_t deviceAddress = 0x50);
    bool init(int port, uint8_t deviceAddress = 0x50);
    void terminate();
    void setDeviceAddress(uint8_t deviceaddress);
    // write page size must be a power of 2, default is 64 (page size for 24LC256 EEPROM)
    void setWritePageSize(uint16_t pageSize);
    bool writeByte(uint16_t eeaddress, uint8_t byte);
    bool writeByteDev(uint8_t deviceaddress, uint16_t eeaddress, uint8_t byte);
    bool write(uint16_t eeAddress, uint8_t *data, size_t size);
    bool writeDev(uint8_t deviceAddress, uint16_t eeAddress, uint8_t *data, size_t size);
    bool sync();
    bool syncDev(uint8_t deviceAddress);
    bool readByte(uint16_t eeAddress, uint8_t *bytePtr);
    bool readByteDev(uint8_t deviceAddress, uint16_t eeAddress, uint8_t *bytePtr);
    bool read(uint16_t eeAddress, uint8_t *data, size_t size);
    bool readDev(uint8_t deviceAddress, uint16_t eeAddress, uint8_t *data, size_t size);

};

inline bool EEPROMMasterDriver::writeByte(uint16_t eeAddress, uint8_t byte) {
    return writeByteDev(deviceAddress, eeAddress, byte);
}
inline bool EEPROMMasterDriver::write(uint16_t eeAddress, uint8_t *data, size_t size)
{
    return writeDev(deviceAddress, eeAddress, data, size);
}
inline bool EEPROMMasterDriver::sync() {
    return syncDev(deviceAddress);
}
inline bool EEPROMMasterDriver::readByte(uint16_t eeAddress, uint8_t *bytePtr) {
    return readByteDev(deviceAddress, eeAddress, bytePtr);
}
inline bool EEPROMMasterDriver::read(uint16_t eeAddress, uint8_t *data, size_t size) {
    return readDev(deviceAddress, eeAddress, data, size);
}
inline bool EEPROMMasterDriver::readByteDev(uint8_t deviceAddress, uint16_t eeAddress, uint8_t *bytePtr)
{
    return readDev(deviceAddress, eeAddress, bytePtr, 1);
}

#endif

