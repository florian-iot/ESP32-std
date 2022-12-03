#include <CompilationOpts.h>

#ifndef INCL_SYSTEM_H
#define INCL_SYSTEM_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <driver/i2c.h>
#include <Wire.h>
#include "Util.h"

class SysPinData {
friend class SystemService;
    int pin;
    const char *service;
    const char *name;
public:
    SysPinData() { pin = -1; service = nullptr; name = nullptr; }
    SysPinData(const char *service, const char *name) { pin = -1; this->service = service; this->name = name; }
    SysPinData(const SysPinData &other) { pin = other.pin; service = other.service; name = other.name; }
    void setPin(int pin);
    int getPin() { return pin; }
    const char *getServiceName() { return service; }
    const char *getName() { return name; }
};

class SysPin {
friend class SystemService;
    int pin;
    SysPinData *pinData;

    SysPin(SysPinData *data) { pin = -1; pinData = data; }
public:
    SysPin() { pin = -1; pinData = nullptr; }
    SysPin(const SysPin &other) { pin = other.pin; pinData = other.pinData; }
    SysPin &operator=(const SysPin &other) { pin = other.pin; pinData = other.pinData; return *this; }
    int getPin() { return pin; }
    operator int() { return pin; }
    void setPin(int pin) { this->pin = pin; pinData->setPin(pin); }
    const char *getName() { return pinData->getName(); };
    const char *getServiceName() { return pinData->getServiceName(); };
};

#endif
