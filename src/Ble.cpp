#include <CompilationOpts.h>

#ifdef USE_BLE

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Ble.h"
#include <NimBLEDevice.h>
#include <NimBLEAdvertisedDevice.h>

BleSensor::BleSensor(const BleAddress &address, const char *name, const char *origName, BleSensorType *type)
: address(address)
{
    this->name = name; // copying into String
    this->origName = origName; // copying into String
    this->type = type;
}

BleSensor::~BleSensor()
{
}

template <class T>
T* BleSensor::as()
{
    return static_cast<T*>(this);
}

// 888    Y88b   d88P 888       888  .d8888b.  8888888b.   .d8888b.   .d8888b.   .d88888b.  
// 888     Y88b d88P  888   o   888 d88P  Y88b 888  "Y88b d88P  Y88b d88P  Y88b d88P" "Y88b 
// 888      Y88o88P   888  d8b  888 Y88b.      888    888 888    888 888    888 888     888 
// 888       Y888P    888 d888b 888  "Y888b.   888    888 888        888        888     888 
// 888        888     888d88888b888     "Y88b. 888    888 888        888  88888 888     888 
// 888        888     88888P Y88888       "888 888    888 888    888 888    888 888 Y8b 888 
// 888        888     8888P   Y8888 Y88b  d88P 888  .d88P Y88b  d88P Y88b  d88P Y88b.Y8b88P 
// 88888888   888     888P     Y888  "Y8888P"  8888888P"   "Y8888P"   "Y8888P88  "Y888888"  
//                                                                                     Y8b  


const char *BleSensorType_LYWSDCGQ::getType()
{
    return "LYWSDCGQ";
}

bool BleSensorType_LYWSDCGQ::isAdvertisementOnly()
{
    return true;
}

bool BleSensorType_LYWSDCGQ::matches(NimBLEAdvertisedDevice* dev)
{
    NimBLEUUID uuid((uint16_t)0xfe95);
    std::string service = dev->getServiceData(uuid);
    return (service.size() > 0 && service.find("\x50\x20\xAA\x01") != service.npos);
}

BleSensor *BleSensorType_LYWSDCGQ::newSensor(const BleAddress &address, const char *name, const char *origName)
{
    return new BleSensor_LYWSDCGQ(address, name, origName, this);
}

BleSensor_LYWSDCGQ::BleSensor_LYWSDCGQ(const BleAddress &address, const char *name, const char *origName, BleSensorType_LYWSDCGQ *type)
: BleSensor(address, name, origName, type)
{
    _hasMeasures = false;
    temp = 99999;
    hum = 99999;
    battPct = 99999;
    rssi = 99999;
}

BleSensor_LYWSDCGQ::~BleSensor_LYWSDCGQ()
{
}

bool BleSensor_LYWSDCGQ::isMeasurementComplete()
{
    return temp != 99999 && hum != 99999; // we won't wait to get battery info 
}

void BleSensor_LYWSDCGQ::clearMeasurements()
{
    _hasMeasures = false;
    temp = 99999;
    hum = 99999;
    battPct = 99999;
    rssi = 99999;
}


void BleSensor_LYWSDCGQ::advertisement(NimBLEAdvertisedDevice* dev)
{
    NimBLEUUID uuid((uint16_t)0xfe95);
    std::string service = dev->getServiceData(uuid);
Serial.printf("    Service length %d\n", service.length());
    if (service.length() < 13) {
        return;
    }
    int16_t type = service[11] + (service[12] << 8);
Serial.printf("    Type: 0x%x\n", type);
    if (type == 0x100d) {
        if (service.length() < 18) {
            return;
        }
        temp = service[14] + (service[15] << 8);
        hum = service[16] + (service[17] << 8);
        _hasMeasures = true;
    } else if (type == 0x1004) {
        if (service.length() < 16) {
            return;
        }
        temp = service[14] + (service[15] << 8);
        _hasMeasures = true;
    } else if (type == 0x1006) {
        if (service.length() < 16) {
            return;
        }
        hum = service[14] + (service[15] << 8);
        _hasMeasures = true;
    } else if (type == 0x100a) {
        if (service.length() < 15) {
            return;
        }
        battPct = service[14];
        _hasMeasures = true;
    }
    if (dev->haveRSSI()) {
        rssi = dev->getRSSI();
    }
}

void BleSensor_LYWSDCGQ::connected()
{
}

bool BleSensor_LYWSDCGQ::hasMeasures()
{
    return _hasMeasures;
}

void BleSensor_LYWSDCGQ::getMeasures(JsonObject *data)
{
    if (temp != 99999) {
        (*data)["temp"] = temp * 100;
    }
    if (hum != 99999) {
        (*data)["hum"] = hum * 10;
    }
    if (battPct != 99999) {
        (*data)["battPct"] = battPct * 10;
    }
    if (rssi != 99999) {
        (*data)["rssi"] = rssi;
    }
}

void BleSensor_LYWSDCGQ::getMeasures(BleSensorMeasures *measures)
{
    measures->temp = 99999 ? 99999 : temp * 100;
    measures->humidity = 99999 ? 99999 : hum * 10;
    measures->sensorBatteryPct = 99999 ? 99999 : battPct * 10;
    measures->sensorRssi = rssi;
}

// 888    Y88b   d88P 888       888  .d8888b.  8888888b.   .d8888b.   .d8888b.  888b     d888 888b     d888  .d8888b.  
// 888     Y88b d88P  888   o   888 d88P  Y88b 888  "Y88b d88P  Y88b d88P  Y88b 8888b   d8888 8888b   d8888 d88P  Y88b 
// 888      Y88o88P   888  d8b  888 Y88b.      888    888 888    888      .d88P 88888b.d88888 88888b.d88888 888    888 
// 888       Y888P    888 d888b 888  "Y888b.   888    888 888    888     8888"  888Y88888P888 888Y88888P888 888        
// 888        888     888d88888b888     "Y88b. 888    888 888    888      "Y8b. 888 Y888P 888 888 Y888P 888 888        
// 888        888     88888P Y88888       "888 888    888 888    888 888    888 888  Y8P  888 888  Y8P  888 888    888 
// 888        888     8888P   Y8888 Y88b  d88P 888  .d88P Y88b  d88P Y88b  d88P 888   "   888 888   "   888 Y88b  d88P 
// 88888888   888     888P     Y888  "Y8888P"  8888888P"   "Y8888P"   "Y8888P"  888       888 888       888  "Y8888P"  


//                 d8888 88888888888 .d8888b.                                                                          
//                d88888     888    d88P  Y88b                                                                         
//               d88P888     888    888    888                                                                         
//              d88P 888     888    888                                                                                
//             d88P  888     888    888                                                                                
//            d88P   888     888    888    888                                                                         
//           d8888888888     888    Y88b  d88P                                                                         
// 88888888 d88P     888     888     "Y8888P"                                                                          


const char *BleSensorType_LYWSD03MMC_ATC::getType()
{
    return "LYWSD03MMC_ATC";
}

bool BleSensorType_LYWSD03MMC_ATC::isAdvertisementOnly()
{
    return true;
}

bool BleSensorType_LYWSD03MMC_ATC::matches(NimBLEAdvertisedDevice* dev)
{
    NimBLEUUID uuid((uint16_t)0x181a);
    std::string service = dev->getServiceData(uuid);
    NimBLEAddress bleAddr = dev->getAddress();
    const uint8_t *addr = bleAddr.getNative(); // don't getNative() directly on result of dev->getAddress(), as it is removed from stack
    const uint8_t macPrefix[3] = { 0xa4, 0xc1, 0x38 };

    return
        service.size() >= 13
        && memcmp(macPrefix, service.c_str(), 3) == 0
        && (addr[0] == (uint8_t)service[5] && addr[1] == (uint8_t)service[4] && addr[2] == (uint8_t)service[3]
            && addr[3] == (uint8_t)service[2] && addr[4] == (uint8_t)service[1] && addr[5] == (uint8_t)service[0]);

    // a4:c1:38:72:0f:33 RSSI -42 "Mijia1"
    //     Service data UUID: 0x181a "A4C138720F3300F3 3B380A9771"
}

BleSensor *BleSensorType_LYWSD03MMC_ATC::newSensor(const BleAddress &address, const char *name, const char *origName)
{
    return new BleSensor_LYWSD03MMC_ATC(address, name, origName, this);
}

BleSensor_LYWSD03MMC_ATC::BleSensor_LYWSD03MMC_ATC(const BleAddress &address,
    const char *name, const char *origName, BleSensorType_LYWSD03MMC_ATC *type)
: BleSensor(address, name, origName, type)
{
    temp = 99999;
    hum = 99999;
    battPct = 99999;
    battVoltage = 99999;
    rssi = 99999;
}

BleSensor_LYWSD03MMC_ATC::~BleSensor_LYWSD03MMC_ATC()
{
}

bool BleSensor_LYWSD03MMC_ATC::isMeasurementComplete()
{
    return temp != 99999; // we always have all measurements
}

void BleSensor_LYWSD03MMC_ATC::clearMeasurements()
{
    temp = 99999;
    hum = 99999;
    battPct = 99999;
    battVoltage = 99999;
    rssi = 99999;
}

void BleSensor_LYWSD03MMC_ATC::advertisement(NimBLEAdvertisedDevice* dev)
{
    NimBLEUUID uuid((uint16_t)0x181a);
    std::string service = dev->getServiceData(uuid);
    // a4:c1:38:72:0f:33 RSSI -42 "Mijia1"
    //     Service data UUID: 0x181a "A4C138720F33 00F3 3B  38    0A97    71"
    //                                             Temp Hum Batt% Batt_mV frame_counter
    //                                0            6    8   9     10      12

    if (service.length() < 13) {
        return;
    }
    temp = (int16_t)((service[6] << 8) + service[7]);
    hum = (uint8_t)service[8];
    battPct = (uint8_t)service[9];
    battVoltage = (int16_t)((service[10] << 8) + service[11]);
    rssi = dev->getRSSI();
}

void BleSensor_LYWSD03MMC_ATC::connected()
{
}

bool BleSensor_LYWSD03MMC_ATC::hasMeasures()
{
    return temp != 99999;
}

void BleSensor_LYWSD03MMC_ATC::getMeasures(JsonObject *data)
{
    if (temp != 99999) {
        (*data)["temp"] = temp * 100;
        (*data)["hum"] = hum;
        (*data)["battPct"] = battPct * 100;
        (*data)["battVoltage"] = battVoltage;
    }
    if (rssi != 99999) {
        (*data)["rssi"] = rssi;
    }
}

void BleSensor_LYWSD03MMC_ATC::getMeasures(BleSensorMeasures *measures)
{
    measures->temp = 99999 ? 99999 : temp * 100;
    measures->humidity = 99999 ? 99999 : hum * 100;
    measures->sensorBatteryPct = 99999 ? 99999 : battPct * 100;
    measures->sensorBatteryVoltage = battVoltage;
    measures->sensorRssi = rssi;
}

// 888    Y88b   d88P 888       888  .d8888b.  8888888b.   .d8888b.   .d8888b.  888b     d888 888b     d888  .d8888b.  
// 888     Y88b d88P  888   o   888 d88P  Y88b 888  "Y88b d88P  Y88b d88P  Y88b 8888b   d8888 8888b   d8888 d88P  Y88b 
// 888      Y88o88P   888  d8b  888 Y88b.      888    888 888    888      .d88P 88888b.d88888 88888b.d88888 888    888 
// 888       Y888P    888 d888b 888  "Y888b.   888    888 888    888     8888"  888Y88888P888 888Y88888P888 888        
// 888        888     888d88888b888     "Y88b. 888    888 888    888      "Y8b. 888 Y888P 888 888 Y888P 888 888        
// 888        888     88888P Y88888       "888 888    888 888    888 888    888 888  Y8P  888 888  Y8P  888 888    888 
// 888        888     8888P   Y8888 Y88b  d88P 888  .d88P Y88b  d88P Y88b  d88P 888   "   888 888   "   888 Y88b  d88P 
// 88888888   888     888P     Y888  "Y8888P"  8888888P"   "Y8888P"   "Y8888P"  888       888 888       888  "Y8888P"  


//          8888888b.  888     888 888     888 Y88b   d88P 
//          888   Y88b 888     888 888     888  Y88b d88P  
//          888    888 888     888 888     888   Y88o88P   
//          888   d88P Y88b   d88P Y88b   d88P    Y888P    
//          8888888P"   Y88b d88P   Y88b d88P     d888b    
//          888          Y88o88P     Y88o88P     d88888b   
//          888           Y888P       Y888P     d88P Y88b  
// 88888888 888            Y8P         Y8P     d88P   Y88b 




const char *BleSensorType_LYWSD03MMC_PVVX::getType()
{
    return "LYWSD03MMC_PVVX";
}

bool BleSensorType_LYWSD03MMC_PVVX::isAdvertisementOnly()
{
    return true;
}

bool BleSensorType_LYWSD03MMC_PVVX::matches(NimBLEAdvertisedDevice* dev)
{
    NimBLEUUID uuid((uint16_t)0x181a);
    std::string service = dev->getServiceData(uuid);

// a4:c1:38:8f:ea:67 RSSI -64 "Mijia2"
//     Service data UUID: 0x181a "67EA8F38C1A4 E608 B110480B4CCF01"

    NimBLEAddress bleAddr = dev->getAddress();
    const uint8_t *addr = bleAddr.getNative(); // don't getNative() directly on result of dev->getAddress(), as it is removed from stack

    return
        service.size() == 15
        // mac prefix
        && (service.c_str()[5] == 0xa4 && service.c_str()[4] == 0xc1 && service.c_str()[3] == 0x38)
        // mac addr in payload
        && (addr[0] == (uint8_t)service[0] && addr[1] == (uint8_t)service[1] && addr[2] == (uint8_t)service[2]
            && addr[3] == (uint8_t)service[3] && addr[4] == (uint8_t)service[4] && addr[5] == (uint8_t)service[5]);
}

BleSensor *BleSensorType_LYWSD03MMC_PVVX::newSensor(const BleAddress &address, const char *name, const char *origName)
{
    return new BleSensor_LYWSD03MMC_PVVX(address, name, origName, this);
}

BleSensor_LYWSD03MMC_PVVX::BleSensor_LYWSD03MMC_PVVX(const BleAddress &address,
    const char *name, const char *origName, BleSensorType_LYWSD03MMC_PVVX *type)
: BleSensor(address, name, origName, type)
{
    clearMeasurements();
}

BleSensor_LYWSD03MMC_PVVX::~BleSensor_LYWSD03MMC_PVVX()
{
}

bool BleSensor_LYWSD03MMC_PVVX::isMeasurementComplete()
{
    return temp != 99999; // we always have all measurements
}

void BleSensor_LYWSD03MMC_PVVX::clearMeasurements()
{
    temp = 99999;
    hum = 99999;
    battPct = 99999;
    battVoltage = 99999;
    counter = 99999;
    gpioStatus = false;
    gpioIsSet = false;
    tempTriggerEvent = false;
    humTriggerEvent = false;
    rssi = 99999;
}

void BleSensor_LYWSD03MMC_PVVX::advertisement(NimBLEAdvertisedDevice* dev)
{
    NimBLEUUID uuid((uint16_t)0x181a);
    std::string service = dev->getServiceData(uuid);
    // a4:c1:38:8f:ea:67 RSSI -64 "Mijia2"
    //     Service data UUID: 0x181a "67EA8F38C1A4 E608 B110 480B     4C     CF   01"
    //                                             6    8    10       12     13   14
    //                                             Temp Hum  BattmV   Batt%  Cnt  Flags

    temp = (int16_t)((uint8_t)service[6] | ((uint8_t)service[7] << 8));
    hum = (int16_t)((uint8_t)service[8] | ((uint8_t)service[9] << 8));
    battVoltage = (int16_t)((uint8_t)service[10] | ((uint8_t)service[11] << 8));
    battPct = (uint8_t)service[12];
    counter = (uint8_t)service[13];
    uint8_t flags = (uint8_t)service[14];
    reedSwitchStatus = !!(flags & 0x01);
    gpioStatus = !!(flags & 0x02);
    gpioIsSet = !!(flags & 0x04);
    tempTriggerEvent = !!(flags & 0x08);
    humTriggerEvent = !!(flags & 0x10);

    rssi = dev->getRSSI();
}

void BleSensor_LYWSD03MMC_PVVX::connected()
{
}

bool BleSensor_LYWSD03MMC_PVVX::hasMeasures()
{
    return temp != 99999;
}

void BleSensor_LYWSD03MMC_PVVX::getMeasures(JsonObject *data)
{
    if (temp != 99999) {
        (*data)["temp"] = temp;
        (*data)["hum"] = hum;
        (*data)["battPct"] = battPct * 100;
        (*data)["battVoltage"] = battVoltage;
        (*data)["counter"] = counter;
        (*data)["digitalStatus"] = reedSwitchStatus;
        (*data)["gpioStatus"] = gpioStatus;
        (*data)["gpioIsSet"] = gpioIsSet;
        (*data)["tempTrig"] = tempTriggerEvent;
        (*data)["humTrig"] = humTriggerEvent;
    }
    if (rssi != 99999) {
        (*data)["rssi"] = rssi;
    }
}

void BleSensor_LYWSD03MMC_PVVX::getMeasures(BleSensorMeasures *measures)
{
    measures->temp = 99999 ? 99999 : temp * 100;
    measures->humidity = 99999 ? 99999 : hum * 100;
    measures->sensorBatteryPct = 99999 ? 99999 : battPct * 100;
    measures->sensorBatteryVoltage = battVoltage;
    measures->sensorRssi = rssi;
    measures->counter = counter;
    measures->gpio1Status = reedSwitchStatus;
    measures->gpio2Status = gpioStatus;
    measures->gpio2IsSettable = gpioIsSet;
    measures->tempTriggerEvent = tempTriggerEvent;
    measures->humTriggerEvent = humTriggerEvent;
}



// 888888b.   888          .d8888b.                            d8b                  
// 888  "88b  888         d88P  Y88b                           Y8P                  
// 888  .88P  888         Y88b.                                                     
// 8888888K.  888  .d88b.  "Y888b.    .d88b.  888d888 888  888 888  .d8888b .d88b.  
// 888  "Y88b 888 d8P  Y8b    "Y88b. d8P  Y8b 888P"   888  888 888 d88P"   d8P  Y8b 
// 888    888 888 88888888      "888 88888888 888     Y88  88P 888 888     88888888 
// 888   d88P 888 Y8b.    Y88b  d88P Y8b.     888      Y8bd8P  888 Y88b.   Y8b.     
// 8888888P"  888  "Y8888  "Y8888P"   "Y8888  888       Y88P   888  "Y8888P "Y8888  

BleService *BleService::thisService;

void BleService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("ble");

    sensorTypes.push_back(new BleSensorType_LYWSDCGQ());
//    sensorTypes.push_back(new BleSensorType_LYWSD03MMC_ATC());
    sensorTypes.push_back(new BleSensorType_LYWSD03MMC_PVVX());

    ServiceCommands *cmd = commandMgr->getServiceCommands("ble");
    initDfa();
    initCommands(cmd);

    // defaults

    isEnabled = true;

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
    sensorScanTimeout = 60000;

    thisService = this;

    // init hardware
}

static void strDump(std::string *dest, std::string &src)
{
    for (auto p = src.begin(); p != src.end(); p++) {
        if (p != src.begin() && (p - src.begin()) % 8 == 0) {
            dest->append(1, ' ');
        }
        char v = *p;
        char h = (v >> 4) & 0x0F;
        dest->append(1, (char)(h < 10 ? '0' + h: 'A' + h - 10));
        h = v & 0xF;
        dest->append(1, (char)(h < 10 ? '0' + h: 'A' + h - 10));
    }
}

void BleService::onResult(NimBLEAdvertisedDevice* dev)
{
    NimBLEAddress addr = dev->getAddress();
    // if (addr.toString().compare("a4:c1:38:72:0f:33") != 0) {
    //     return;
    // }
    // if (!dev->haveName()) {
    //     return;
    // }
    String str;
    str.concat(AddressToString(addr).c_str());
    str.concat(" RSSI ");
    str.concat(dev->getRSSI());
    str.concat(" \"");
    if (dev->haveName()) {
        str.concat(dev->getName().c_str());
    } else {
        str.concat("N/A");
    }
    str.concat("\"");
    if (dev->haveManufacturerData()) {
        std::string manufacturerStr;
        std::string manufacturer = dev->getManufacturerData();
        strDump(&manufacturerStr, manufacturer);
        str.concat(" Manufacturer data: ");
        str.concat(manufacturerStr.c_str());
    }
    str.concat("\n");
    int serviceCnt = dev->getServiceUUIDCount();
    for (int s = 0; s < serviceCnt; s++) {
        NimBLEUUID serviceUuid = dev->getServiceUUID(s);
        str.concat("    Service UUID: ");
        str.concat(AddressToString(serviceUuid).c_str());
        str.concat("\n");
    }
    int serviceDataCnt = dev->getServiceDataCount();
    for (int d = 0; d < serviceDataCnt; d++) {
        NimBLEUUID serviceDataUuid = dev->getServiceDataUUID(d);
        std::string data = dev->getServiceData(d);
        std::string dataStr;
        strDump(&dataStr, data);
        str.concat("    Service data UUID: ");
        str.concat(AddressToString(serviceDataUuid).c_str());
        str.concat(" \"");
        str.concat(dataStr.c_str());
        str.concat("\"\n");
    }

    if (!isSensorScan) {
Serial.print(str.c_str());

        for (auto st = sensorTypes.begin(); st != sensorTypes.end(); st++) {
            if ((*st)->matches(dev)) {
                // see if we have it already in our scan list
                bool isInList = false;
                for (auto scanItem = scanList.begin(); scanItem != scanList.end(); scanItem++) {
                    if ((*scanItem)->address == BleAddress(dev->getAddress())) {
                        isInList = true;
                        break;
                    }
                }
                if (!isInList) {
Serial.print(str.c_str());
Serial.printf("Recognized %s of type %s\n", AddressToString(dev->getAddress()).c_str(), (*st)->getType());
                    scanList.push_back(new BleScanItem(BleAddress(dev->getAddress()), *st, dev->getName().c_str()));
                }
            }
        }
    } else { // sensor scan
        BleAddress addr(dev->getAddress());
        for (auto sl = sensorList.begin(); sl != sensorList.end(); sl++) {
            if ((*sl)->getAddress() == addr) {
Serial.printf("Got advertisement for %s: ", str.c_str());
                (*sl)->advertisement(dev);
DynamicJsonBuffer buf;
JsonObject &obj = buf.createObject();
(*sl)->getMeasures(&obj);
str.clear();
obj.prettyPrintTo(str);
Serial.println(str);
                break;
            }
        }
    }

}

void BleService::staticScanComplete(NimBLEScanResults results)
{
    thisService->scanComplete(results);
}

void BleService::scanComplete(NimBLEScanResults results)
{
Serial.println("Scan complete");
    dfa.handleInput(SCAN_COMPLETED);
}

class MyClientCallback : public NimBLEClientCallbacks {
    void onConnect(BLEClient* pclient) {
        Serial.printf("onConnect");
    }
    void onDisconnect(BLEClient* pclient, int reason) {
        Serial.println("onDisconnect");
    }
    uint32_t onPassKeyRequest() {
        Serial.printf("Client PassKeyRequest\n");
        return 123456; 
    }
    bool onConfirmPIN(uint32_t passKey) {
        Serial.printf("The passkey YES/NO number: %d\n", passKey);
        return true; 
    }
    void onAuthenticationComplete(NimBLEConnInfo &connInfo){
        Serial.printf("Starting BLE work!\n");
    }
};

void BleService::initDfa()
{
    dfa.init(eventLoop, nullptr /* or logger, to log */, STARTING);

    dfa.onInput([this](Dfa *dfa, Dfa::State state, Dfa::Input input) {
Serial.printf("Ble DFA: in state %s, input %s\n", dfa->stateName(state), dfa->inputName(input));

        if (state.is(STARTING)) {
            if (input.is(Dfa::Input::ENTER_STATE)) { // wait a bit, initialize BLE, and go to IDLE
                dfa->setStateTimeout(1000);
                return dfa->noTransition();
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                NimBLEDevice::init("");
                scan = NimBLEDevice::getScan();
                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }
        } else if (state.is(IDLE)) {
            if (input.is(START_SCAN)) {
Serial.println("Starting scan");
                isSensorScan = false;
                // scan->setFilterPolicy(BLE_HCI_SCAN_FILT_NO_WL); // do not use whitelist
                // scan->setDuplicateFilter(true);
                scan->setAdvertisedDeviceCallbacks(this, false); // we don't want duplicates

                scan->setInterval(100); // default 100
                scan->setWindow(100);  // default 100, less or equal setInterval value
                scan->setActiveScan(true);
               
//                scan->setMaxResults(0); // do not store results, only use the callback
                scan->start(30, staticScanComplete, false);
                return dfa->transitionTo(WAIT_SCAN_COMPLETED);
            } else if (input.is(START_SENSOR_SCAN)) {
                return dfa->transitionTo(SENSOR_SCAN);
            } else {
                return dfa->transitionError();
            }
        } else if (state.is(WAIT_SCAN_COMPLETED)) {
            if (input.is(SCAN_COMPLETED)) {
                scan->clearResults(); // delete results from BLEScan buffer to release memory
                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }
        } else if (state.is(SENSOR_SCAN)) {
            if (input.is(Dfa::Input::ENTER_STATE)) {
Serial.println("Starting sensor scan");
                isSensorScan = true;
#if 0
                // clear current white list
                for (int n = NimBLEDevice::getWhiteListCount(); n > 0; n--) {
                    NimBLEDevice::whiteListRemove(n - 1);
                }
                // add to white list
                for (auto sl = sensorList.begin(); sl != sensorList.end(); sl++) {
                    NimBLEDevice::whiteListAdd((*sl)->getAddress().toNimBLEAddress());
                }
#endif
                // clear current measures
                for (auto s = sensorList.begin(); s != sensorList.end(); s++) {
                    (*s)->clearMeasurements();
                }
                sensorScanStartTm = millis();
//                scan->setFilterPolicy(BLE_HCI_SCAN_FILT_USE_WL); // use whitelist
//                scan->setDuplicateFilter(false);
                scan->setAdvertisedDeviceCallbacks(this, true); // we want duplicates (sensors will update measures)
                scan->setActiveScan(false);
//                scan->setMaxResults(0); // do not store results, only use the callback
                scan->start(0, staticScanComplete, false); // continuous scan, until stopped
                return dfa->transitionTo(SENSOR_SCAN_WAIT, 1000);
            } else {
                return dfa->transitionError();
            }
        } else if (state.is(SENSOR_SCAN_WAIT)) {
            if (input.is(Dfa::Input::TIMEOUT)) {
                bool isComplete = true;
                for (auto s = sensorList.begin(); s != sensorList.end(); s++) {
                    if (!(*s)->isMeasurementComplete()) {
                        isComplete = false;
                        break;
                    }
                }
                if (isComplete || ((long)millis() - sensorScanStartTm) > sensorScanTimeout) {
                    scan->stop();
                    for (int i = 0; i < sensorScanTerminated.size(); i++) {
                        sensorScanTerminated.get(i)();
                    }
                    sensorScanTerminated.clear();
                    return dfa->transitionTo(IDLE);
                } else {
                    dfa->setStateTimeout(1000);
                    return dfa->noTransition();
                }
            } else if (input.is(STOP_SENSOR_SCAN)) {
                scan->stop();
                for (int i = 0; i < sensorScanTerminated.size(); i++) {
                    sensorScanTerminated.get(i)();
                }
                sensorScanTerminated.clear();
                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }
        } else {
            return dfa->transitionError();
        }
    });
}

void BleService::initCommands(ServiceCommands *cmd)
{
    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("scan", true)
        .cmdOn("scan")
        .helpOn("--> start a scan")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            dfa.handleInput(START_SCAN);
            *msg = "Starting scan";
            return true;
        })
    );

    cmd->registerStringData(ServiceCommands::StringDataBuilder("scanList", true)
        .cmd("scanList")
        .help("--> List of scanned and recognized devices")
        .isPersistent(false)
        .getFn([this](String *val) {
            val->clear();
            for (auto sl = scanList.begin(); sl != scanList.end(); sl++) {
                val->concat("\n        ");
                val->concat(AddressToString((*sl)->address.toNimBLEAddress()).c_str());
                val->concat(" \"");
                val->concat((*sl)->name.c_str());
                val->concat("\" of type ");
                val->concat((*sl)->type->getType());
            }
        })
    );

    cmd->registerIntData(ServiceCommands::IntDataBuilder("sensor.scanTimeout", true)
        .cmd("sensor.scanTimeout")
        .help("--> Timeout in milliseconds for sensor scanning")
        .vMin(1)
        .vMax(3600 * 1000)
        .ptr(&sensorScanTimeout)
    );

    cmd->registerStringData(ServiceCommands::StringDataBuilder("sensor.add", true)
        .cmd("sensor.add")
        .help("sensor.add <address> --> Add a sensor to monitor, out of the ones listed and recognized during the scan (command scanList)")
        .isPersistent(false)
        .setFn([this](const String &val, bool isLoading, String *msg) {
            BleAddress addr(val.c_str());
            if (addr.isNull()) {
                *msg = "Incorrect address syntax";
                return true;
            }

            // check that it's one of the listed ones
            BleScanItem *scanItem = nullptr;
            for (auto si = scanList.begin(); si != scanList.end(); si++) {
                if ((*si)->address == addr) {
                    scanItem = *si;
                    break;
                }
            }
            if (scanItem == nullptr) {
                *msg = "Address not found in the list of scanned devices";
                return true;
            }

            // check that it's not already registered
            bool isRegistered = false;
            for (auto si = sensorList.begin(); si != sensorList.end(); si++) {
                if ((*si)->getAddress() == addr) {
                    isRegistered = true;
                    break;
                }
            }
            if (isRegistered) {
                *msg = "This sensor is already registered";
                return true;
            } else {
                const char *name = scanItem->name.c_str();
                // do we have a name composed only of spaces or empty? Use mac address.
                if (scanItem->name.isEmpty()) {
                    name = AddressToString(scanItem->address.toNimBLEAddress()).c_str();
                } else {
                    bool isNameUsable = false;
                    for (const char *p = name; *p != '\0'; p++) {
                        if (*p != ' ') {
                            isNameUsable = true;
                            break;
                        }
                    }
                    if (!isNameUsable) {
                        name = AddressToString(scanItem->address.toNimBLEAddress()).c_str();
                    }
                }

                sensorList.push_back(scanItem->type->newSensor(addr, name, scanItem->name.c_str()));
            }

            *msg = "Added to sensor list";
            return true;
        })
    );

    cmd->registerJsonData(ServiceCommands::JsonDataBuilder("sensor.list", true)
        .cmd("sensor.list")
        .help("--> List of registered sensors")
        .isPersistent(true)
        .getFn([this](JsonBuffer &buf) {
            JsonArray *val = &buf.createArray();
            for (auto sl = sensorList.begin(); sl != sensorList.end(); sl++) {
                JsonObject &s = buf.createObject();
                s["name"] = (*sl)->getName();
                s["addr"] = (char *)AddressToString((*sl)->getAddress().toNimBLEAddress()).c_str();
                s["type"] = (*sl)->getType()->getType();
                val->add(s);
            }
            return JsonVariant(*val);
        })
        .setFn([this](const JsonVariant &val, bool isLoading, String *msg) {
            for (int i = 0; i < sensorList.size(); i++) {
                delete sensorList[i];
            }
            sensorList.clear();
            JsonArray &list = val.as<JsonArray>();
            for (int i = 0; i < list.size(); i++) {
                BleSensorType *sensorType = nullptr;
                for (int t = 0; t < sensorTypes.size(); t++) {
                    if (strcmp(sensorTypes[t]->getType(), list[i]["type"].as<char *>()) == 0) {
                        sensorType = sensorTypes[t];
                        break;
                    }
                }
                if (sensorType != nullptr) {
                    BleAddress addr(list[i]["addr"].as<char *>());
                    const char *name = list[i]["name"].as<char *>();
                    if (!addr.isNull() && name != nullptr && name[0] != '\0') {
                        sensorList.push_back(sensorType->newSensor(addr, name, ""));
                    }
                }
            }
            return true;
        })
    );

    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("sensor.scan", true)
        .cmd("sensor.scan")
        .help("--> start or stop sensor scanning")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (val) {
                if (dfa.getState() == WAIT_SCAN_COMPLETED) {
                    scan->stop();
                    dfa.queueInputForState(START_SENSOR_SCAN, IDLE);
                    *msg = "Request for starting sensor scan queued";
                } else if (dfa.getState() == IDLE) {
                    dfa.handleInput(START_SENSOR_SCAN);
                    *msg = "Sensor scan started";
                } else {
                    *msg = "Incorrect state: ";
                    msg->concat(dfa.stateName(dfa.getState()));
                }                
            } else {
                if (dfa.getState() == SENSOR_SCAN_WAIT) {
                    dfa.handleInput(STOP_SENSOR_SCAN);
                    *msg = "Sensor scan stopped";
                } else {
                    *msg = "Incorrect state: ";
                    msg->concat(dfa.stateName(dfa.getState()));
                }
            }
            return true;
        })
    );

#if 0
    // room assistant compagnion app on iPhone

    cmd->registerStringData(ServiceCommands::StringDataBuilder("sensor.room-assistant", true)
        .cmd("sensor.room-assistant")
        .help("sensor.room-assistant <address> --> Try to connect and identify iPhone running room-assistant app")
        .isPersistent(false)
        .setFn([this](const String &val, bool isLoading, String *msg) {
            BleAddress addr(val.c_str());
            if (addr.isNull()) {
                *msg = "Incorrect address syntax";
                return true;
            }
            Serial.printf("Connecting to iPhone to check for room-assistant at %s\n", std::string(addr.toNimBLEAddress()).c_str());


            NimBLEAddress a(val.c_str(), BLE_ADDR_RANDOM);
NimBLEDevice::whiteListAdd(a);
            NimBLEClient *client = NimBLEDevice::createClient(a);
            if (client == nullptr) {
                *msg = "Couldn't create client object";
                return true;
            }
            client->setConnectTimeout(10);

// May need this:
// There are certain rules and formulae that the parameters must follow. If the parameters do not
// comply with all of these rules, the parameter request may be rejected, or the stability and the
// performance of the connection may be compromised.

//     Interval Min ≥ 15 ms (multiples of 15 ms)
//     Interval Min + 15 ms ≤ Interval Max (Interval Max == 15 ms is allowed)
//     Interval Max * (Slave Latency + 1) ≤ 2 seconds
//     Interval Max * (Slave Latency + 1) * 3 < connSupervisionTimeout
//     Slave Latency ≤ 30
//     2 seconds ≤ connSupervisionTimeout ≤ 6 seconds
//
// https://developer.apple.com/library/archive/qa/qa1931/_index.html


            bool rc = client->connect();
            if (!rc) {
                *msg = "Couldn't connect to ";
                msg->concat(AddressToString(a).c_str());
                NimBLEDevice::deleteClient(client);
                return true;
            }
            Serial.println("Connected!");
            NimBLERemoteService *service = client->getService("5403c8a7-5c96-47e9-9ab8-59e373d875a7");
            if (service == nullptr) {
                *msg = "Couldn't get service";
                NimBLEDevice::deleteClient(client);
                return true;
            }
            Serial.println("Got service!");
            // NimBLERemoteCharacteristic *charact = service->getCharacteristic("D1338ACE-002D-44AF-88D1-E57C12484966");
            NimBLERemoteCharacteristic *charact = service->getCharacteristic("21C46F33-E813-4407-8601-2AD281030052");
            if (charact == nullptr) {
                *msg = "Couldn't get characteristic";
                NimBLEDevice::deleteClient(client);
                return true;
            }
            Serial.println("Got characteristics!");
            std::string charVal = charact->readValue();
            Serial.printf("Got value of characteristic, length %d: \"%s\"\n", charVal.length(), charVal.c_str());

// 40:80:7a:ca:a4:f2
// ble sensor.room-assistant 7d:1f:ee:4b:55:9e
// ble sensor.room-assistant 50:78:b1:5f:b8:2a

            NimBLEDevice::deleteClient(client);

            *msg = charVal.c_str();
            return true;
        })
    );
#endif


    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("mijia:pin", true)
        .cmd("mijia:pin")
        .help("--> Set or clear GPIO on Mijia temperature sensor")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *str) {

            NimBLEClient* pClient = NimBLEDevice::createClient();
            Serial.printf(" - Created client\n");

            pClient->setClientCallbacks(new MyClientCallback(), true);

            // Connect to the remove BLE Server.
            // Mijia2 NimBLEAddress mijia("a4:c1:38:8f:ea:67");
            NimBLEAddress mijia("A4:C1:38:36:A7:DA");
            bool rc = pClient->connect(mijia);
            if (!rc) {
                Serial.printf(" - Could not connect to server\n");
                *str = "Could not connect to server";
                return true;
            }
            Serial.printf(" - Connected to server\n");

            // Obtain a reference to the service we are after in the remote BLE server.
            NimBLEUUID serviceUUID("00001f10-0000-1000-8000-00805f9b34fb");
            BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
            if (pRemoteService == nullptr) {
                Serial.printf("Failed to find our service UUID: %s\n", serviceUUID.toString().c_str());
                *str = "Failed to find service UUID";
                pClient->disconnect();
                return true;
            }
            Serial.printf(" - Found our service\n");

            // Obtain a reference to the characteristic in the service of the remote BLE server.
            NimBLEUUID charUUID("00001f1f-0000-1000-8000-00805f9b34fb");
            NimBLERemoteCharacteristic *pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
            if (pRemoteCharacteristic == nullptr) {
                Serial.printf("Failed to find our characteristic UUID: %s\n", charUUID.toString().c_str());
                *str = "Failed to find characteristic";
                pClient->disconnect();
                return true;
            }
            Serial.printf(" - Found our characteristic\n");

            // // Read the value of the characteristic.
            std::string value = pRemoteCharacteristic->readValue();
            std::string dump;
            strDump(&dump, value);
            Serial.printf("The characteristic value was: \"%s\"\n", dump.c_str());
            
            uint8_t buf[2];
            buf[0] = 0x45;
            buf[1] = val ? 1 : 0;
            rc = pRemoteCharacteristic->writeValue(buf, 2, false);
            if (!rc) {
                Serial.printf("Failed to write %d\n", buf[1]);
            } else {
                Serial.printf("Wrote %d\n", buf[1]);
            }

            pClient->disconnect();
            NimBLEDevice::deleteClient(pClient);
            *str = "Command sent";
            return true;

        })
    );


}


BleSensor *BleService::getSensor(const char *addr)
{
    BleAddress bleAddress(addr);
    for (auto sl = sensorList.begin(); sl != sensorList.end(); sl++) {
        if ((*sl)->getAddress() == bleAddress) {
            return *sl;
        }
    }
    return nullptr;
}

bool BleService::startSensorScan(std::function<void()> scanTerminatedCallback)
{
    if (scanTerminatedCallback != nullptr) {
        sensorScanTerminated.add(scanTerminatedCallback);
    }
    if (dfa.getState() == IDLE) {
        dfa.handleInput(START_SENSOR_SCAN);
        return true;
    } else if (dfa.getState() == WAIT_SCAN_COMPLETED) {
        scan->stop();
        dfa.queueInputForState(START_SENSOR_SCAN, IDLE);
        return true;
    } else {
        dfa.queueInputForState(START_SENSOR_SCAN, IDLE);
        return true;
    }
}


#endif
