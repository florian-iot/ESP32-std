#include <CompilationOpts.h>

#ifdef USE_BLE
#ifndef INCL_BLE_H
#define INCL_BLE_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "NimBLEDevice.h"

#define AddressToString(addr) std::string(addr)

// NimBLEAddress is not mutable and not easily copiable... hence this class
class BleAddress {
    ble_addr_t address;
public:
    BleAddress(ble_addr_t addr) : address(addr) { }
    BleAddress(const NimBLEAddress &addr) {
        memcpy(address.val, addr.getNative(), 6);
        address.type = addr.getType();
    }
    BleAddress(const char *str) {
        if (str == nullptr) {
            for (int i = 0; i < 6; i++) {
                address.val[i] = 0;
            }
            address.type = 0;
        } else {
            NimBLEAddress addr(str);
            memcpy(address.val, addr.getNative(), 6);
            address.type = addr.getType();
        }
    }
    BleAddress(const BleAddress &other) { address = other.address; }
    BleAddress &operator=(const BleAddress &other) {
        memcpy(address.val, other.address.val, 6);
        address.type = other.address.type;
        return *this;
    }
    bool isNull() const {
        uint8_t n[6] = {0,0,0,0,0,0};
        return memcmp(address.val, n, 6) == 0;
    }

    bool operator==(const BleAddress &other) const { return memcmp(address.val, &other.address.val, 6) == 0 /*&& address.type == other.address.type*/; }
    bool operator!=(const BleAddress &other) const { return !((*this) == other); }
    NimBLEAddress toNimBLEAddress() const { return NimBLEAddress(address); }
};

struct BleSensorMeasures {
    enum Measure {
        TEMP = (1<<1),
        HUMIDITY = (1<<2),
        SENSOR_BATTERY_PCT = (1<<3),
        SENSOR_BATTERY_VOLTAGE = (1<<4),
        SENSOR_RSSI = (1<<5),
        COUNTER = (1<<6),
        GPIO_1_STATUS = (1<<7),
        GPIO_2_STATUS = (1<<8),
        GPIO_2_IS_SETTABLE = (1<<9),
        TEMP_TRIGGER_EVENT = (1<<10),
        HUM_TRIGGER_EVENT = (1<<11)
    };
    int temp; // milli °C
    int humidity; // 0.01 pct
    int sensorRssi;
    int sensorBatteryPct; // 0.01 pct
    int sensorBatteryVoltage; // mV
    int counter;
    bool gpio1Status;
    bool gpio2Status;
    bool gpio2IsSettable;
    bool tempTriggerEvent;
    bool humTriggerEvent;

    // TODO
    // uint64_t measureIsSet;

    BleSensorMeasures(): temp(99999), humidity(99999), sensorRssi(99999),
        sensorBatteryPct(99999), sensorBatteryVoltage(99999),
        counter(99999),
        gpio1Status(false), gpio2Status(false), gpio2IsSettable(false),
        tempTriggerEvent(false), humTriggerEvent(false)
    { }

    int getIntMeasure(Measure m) {
        switch (m) {
            case TEMP: return temp;
            case HUMIDITY: return humidity;
            case SENSOR_BATTERY_PCT: return sensorBatteryPct;
            case SENSOR_BATTERY_VOLTAGE: return sensorBatteryVoltage;
            case SENSOR_RSSI: return sensorRssi;
            case COUNTER: return counter;
            case GPIO_1_STATUS: return gpio1Status;
            case GPIO_2_STATUS: return gpio2Status;
            case GPIO_2_IS_SETTABLE: return gpio2IsSettable;
            case TEMP_TRIGGER_EVENT: return tempTriggerEvent;
            case HUM_TRIGGER_EVENT: return humTriggerEvent;
            default: return 99999;
        }
    }
};

class BleSensor;

class BleSensorType {
public:
    virtual const char *getType() = 0;
    virtual bool isAdvertisementOnly() = 0;
    virtual bool matches(NimBLEAdvertisedDevice* dev) = 0;
    virtual BleSensor *newSensor(const BleAddress &address, const char *name, const char *origName) = 0;
};

class BleSensor {
    BleAddress address;
    String name;
    String origName;
    BleSensorType *type;
public:
    BleSensor(const BleAddress &address, const char *name, const char *origName, BleSensorType *type);
    virtual ~BleSensor();

    const BleAddress &getAddress() { return address; }
    const String &getName() { return name; }
    BleSensorType *getType() { return type; }

    template <class T>
    T* as();

    // whether more advertisements are needed to complete the measurements
    virtual bool isMeasurementComplete() = 0;
    virtual void clearMeasurements() = 0;
    virtual void advertisement(NimBLEAdvertisedDevice* dev) = 0;
    virtual void connected(/*todo*/) = 0;
    virtual bool hasMeasures() = 0;
    virtual void getMeasures(JsonObject *data) = 0;
    virtual void getMeasures(BleSensorMeasures *measures) = 0;
};

class BleSensorType_LYWSDCGQ: public BleSensorType {    
public:
    virtual const char *getType();
    virtual bool isAdvertisementOnly();
    virtual bool matches(NimBLEAdvertisedDevice* dev);
    virtual BleSensor *newSensor(const BleAddress &address, const char *name, const char *origName);
};

class BleSensorType_LYWSD03MMC: public BleSensorType {    
public:
    virtual const char *getType();
    virtual bool isAdvertisementOnly();
    virtual bool matches(NimBLEAdvertisedDevice* dev);
    virtual BleSensor *newSensor(const BleAddress &address, const char *name, const char *origName);
};

class BleSensorType_LYWSD03MMC_PVVX: public BleSensorType {    
public:
    virtual const char *getType();
    virtual bool isAdvertisementOnly();
    virtual bool matches(NimBLEAdvertisedDevice* dev);
    virtual BleSensor *newSensor(const BleAddress &address, const char *name, const char *origName);
};

class BleSensorType_LYWSD03MMC_ATC: public BleSensorType {    
public:
    virtual const char *getType();
    virtual bool isAdvertisementOnly();
    virtual bool matches(NimBLEAdvertisedDevice* dev);
    virtual BleSensor *newSensor(const BleAddress &address, const char *name, const char *origName);
};

class BleSensor_LYWSDCGQ: public BleSensor {
    bool _hasMeasures;
    int temp;
    int hum;
    int battPct;
    int rssi;
    friend class BleSensorType_LYWSDCGQ;
    BleSensor_LYWSDCGQ(const BleAddress &address, const char *name, const char *origName, BleSensorType_LYWSDCGQ *type);
public:
    virtual ~BleSensor_LYWSDCGQ();
    virtual bool isMeasurementComplete();
    virtual void clearMeasurements();
    virtual void advertisement(NimBLEAdvertisedDevice* dev);
    virtual void connected(/*todo*/);
    virtual bool hasMeasures();
    virtual void getMeasures(JsonObject *data);
    virtual void getMeasures(BleSensorMeasures *measures);
};

class BleSensor_LYWSD03MMC_ATC: public BleSensor {
    int temp;
    int hum;
    int battPct;
    int battVoltage;
    int rssi;
    friend class BleSensorType_LYWSD03MMC_ATC;
    BleSensor_LYWSD03MMC_ATC(const BleAddress &address, const char *name, const char *origName, BleSensorType_LYWSD03MMC_ATC *type);
public:
    virtual ~BleSensor_LYWSD03MMC_ATC();
    virtual bool isMeasurementComplete();
    virtual void clearMeasurements();
    virtual void advertisement(NimBLEAdvertisedDevice* dev);
    virtual void connected(/*todo*/);
    virtual bool hasMeasures();
    virtual void getMeasures(JsonObject *data);
    virtual void getMeasures(BleSensorMeasures *measures);
};

class BleSensor_LYWSD03MMC_PVVX: public BleSensor {
    int temp;
    int hum;
    int battPct;
    int battVoltage;
    int counter;
    bool reedSwitchStatus;
    bool gpioStatus;
    bool gpioIsSet;
    bool tempTriggerEvent;
    bool humTriggerEvent;

    int rssi;
    friend class BleSensorType_LYWSD03MMC_PVVX;
    BleSensor_LYWSD03MMC_PVVX(const BleAddress &address, const char *name, const char *origName, BleSensorType_LYWSD03MMC_PVVX *type);
public:
    virtual ~BleSensor_LYWSD03MMC_PVVX();
    virtual bool isMeasurementComplete();
    virtual void clearMeasurements();
    virtual void advertisement(NimBLEAdvertisedDevice* dev);
    virtual void connected(/*todo*/);
    virtual bool hasMeasures();
    virtual void getMeasures(JsonObject *data);
    virtual void getMeasures(BleSensorMeasures *measures);
};

struct BleScanItem {
    BleAddress address;
    BleSensorType *type;
    String name;

    BleScanItem(const BleAddress &addr, BleSensorType *type, const char *name) : address(addr), type(type), name(name) { }
    BleScanItem &operator=(const BleScanItem &other) { address = other.address; type = other.type; name = other.name; return *this; }
};

class BleService: public NimBLEAdvertisedDeviceCallbacks {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;

    NimBLEScan *scan;
    std::vector<BleSensorType *> sensorTypes;
    std::vector<BleScanItem *> scanList;
    std::vector<BleSensor *> sensorList;
    int sensorScanTimeout; // timeout for sensor scanning, in millis

    bool isSensorScan; // true if the current scan does sensor scan, as opposed to all devices scan
    long sensorScanStartTm;
    CallbackList<void()> sensorScanTerminated;

    /**
     * @brief Called when a new scan result is detected.
     *
     * As we are scanning, we will find new devices.  When found, this call back is invoked with a reference to the
     * device that was found.  During any individual scan, a device will only be detected one time.
     */
    virtual void onResult(NimBLEAdvertisedDevice* advertisedDevice);

    static BleService *thisService;
    static void staticScanComplete(NimBLEScanResults results);
    void scanComplete(NimBLEScanResults results);

    Dfa dfa;
    Dfa::State STARTING = dfa.nextState("STARTING");
    Dfa::Input START_SCAN = dfa.nextInput("START_SCAN");
    Dfa::Input SCAN_COMPLETED = dfa.nextInput("SCAN_COMPLETED");
    Dfa::Input START_SENSOR_SCAN = dfa.nextInput("START_SENSOR_SCAN");
    Dfa::Input STOP_SENSOR_SCAN = dfa.nextInput("STOP_SENSOR_SCAN");
    Dfa::State IDLE = dfa.nextState("IDLE");
    Dfa::State WAIT_SCAN_COMPLETED = dfa.nextState("WAIT_SCAN_COMPLETED");
    Dfa::State SENSOR_SCAN = dfa.nextState("SENSOR_SCAN");
    Dfa::State SENSOR_SCAN_WAIT = dfa.nextState("SENSOR_SCAN_WAIT");

    void initCommands(ServiceCommands *cmd);
    void initDfa();

    bool isEnabled;
public:
    BleSensor *getSensor(const char *addr);
    // true if scan started, planned for starting, or currently running. False if not the right state.
    bool startSensorScan(std::function<void()> scanTerminated = nullptr);
};

#endif
#endif