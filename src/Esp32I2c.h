#include <CompilationOpts.h>

#ifdef USE_ESP32I2C
#ifndef INCL_ESP32I2C_H
#define INCL_ESP32I2C_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "SystemService.h"
#include <driver/i2c.h>
#include <NimBLEAddress.h>

/***********************
 * 
 * I2C protocol for reading measurements from a remote esp32
 * 
 * 0. Set parameters
 * - Duration between scans
 * - Sleep mode with auto-wakeup timeout
 * - Request new scan, irrespective of duration between scans
 * 
 * Write
 * Byte 0: request code 0x81
 * Byte 1-4: set duration between scans, in millis
 * Byte 5-8: >0 for sleep mode, indicates auto-wakeup timeout in millis
 * Byte 9: Bit 0: 1 for requesting new scan, else 0.
 * 
 * Followed by a read:
 * Byte 0: Version of response.
 *      1: Version 1, follows what's described here. Next versions can only have longer lengths.
 * 8 bytes (to keep some forward compatibility)
 * Byte 0: Version of esp32 remote
 * 
 * 
 * 1. Write config data to remote esp32
 * 
 * Write
 * Byte 0: request code 0x01
 * Byte 1: type of device
 *     #define BLE_DEVICE_NOT_SPECIFIED 0
 *     #define BLE_DEVICE_LYWSDCGQ 1
 *     #define BLE_DEVICE_LYWSD03MMC_PVVX 2
 *     Others: to be defined.
 *     If BLE_DEVICE_NOT_SPECIFIED, the matching of device scan data is supposed to determine device type.
 * Bytes 2 - 7: MAC address of BLE device
 * Only public MAC addresses are used
 * 
 * The call can be repeated to indicate multiple addresses. (Currently not implemented,
 * a single address is remembered)
 * The list of addresses is limited in length to 50 entries (old entries are purged),
 * and does not survive a reboot.
 * The last written MAC address is the one whose data will be read with a read
 * measurements call.
 * 
 * 
 * 2. Read the available measurements
 * 
 * First write config data, for MAC address to get measures from.
 * Then read. One read for each writing of config data.
 * 
 * The remote esp32 will perform BLE scan every 5 minutes. Received data
 * will correspond to the latest scan result (or 99999 values if the scan
 * hasn't been performed yet.)
 * 
 * Read 1 byte
 * Byte 0: number of available measures
 * 
 * Read N * 5 bytes, N == number of available measures. Each measure is:
 * Byte 0: type of measure
 *     1: temp, in 1/1000 °C
 *     2: humidity, in 1/100 of %
 *     3: batteryPct, in 1/100 of %
 *     4: batteryVoltage, in mV
 * Byte 1 - 4: value, int32_t, lsb first
 * 
 */

#define BLE_DEVICE_NOT_SPECIFIED 0
#define BLE_DEVICE_LYWSDCGQ 1
#define BLE_DEVICE_LYWSD03MMC_PVVX 2

class Esp32I2cService {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, SystemService *systemService, LogMgr *logMgr);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;
    SystemService *system;

    Dfa dfa;

    Dfa::Input ENABLE = dfa.nextInput("ENABLE");
    Dfa::Input DISABLE = dfa.nextInput("DISABLE");
    Dfa::Input REACQUIRE = dfa.nextInput("REACQUIRE");

    Dfa::State INITIAL = dfa.nextState("INITIAL");
    Dfa::State CONFIGURING = dfa.nextState("CONFIGURING");
    Dfa::State READING_VERSION = dfa.nextState("READING_VERSION");
    Dfa::State WRITE_REQUEST = dfa.nextState("WRITE_REQUEST");
    Dfa::State READING = dfa.nextState("READING");
    Dfa::State IDLE = dfa.nextState("IDLE");
    Dfa::State COMM_DISABLED = dfa.nextState("COMM_DISABLED");

    void initCommands(ServiceCommands *cmd);
    void initDfa();

    bool isEnabled;
    i2c_port_t i2cPort;
    int i2cAddress;
    NimBLEAddress bleAddress;
    int measurementPeriodSecs;
    int32_t remoteBleScanDuration;

    uint8_t remoteVersion;
    bool isI2cRetrieved;
    int temp;
    int humidity;
    int batteryPct;
    int batteryVoltage;
    int rssi;
    bool isMeasured;
    // don't change this while we're IDLE and waiting for nextMeasurementTime, because this depends on the deep sleep time of remote esp32
    long nextMeasurementTime;

    void clearMeasures();
    bool writeParams(int32_t scanDurationMillis, int32_t sleepDurationSecs, bool doNewScan);
    bool readVersion();
    bool writeInitData();
    bool writeRequest();
    bool readMeasures();

public:
    void getMeasures(int *envTemp, int *envHumidity, int *sensorBatteryPct, int *sensorRssi);
    
};

#endif
#endif