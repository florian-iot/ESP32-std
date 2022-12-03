#include <CompilationOpts.h>

#ifdef USE_MQTT
#ifndef INCL_MQTT_H
#define INCL_MQTT_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <AsyncMqttClient.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "WifiServiceAsync.h"

class MqttService {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, WifiAsyncService *wifi, LogMgr *logMgr);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;
    WifiAsyncService *wifi;

    void initCommands(ServiceCommands *cmd);

    bool isEnabled;
    String topic;
    String prefix;
    String ipAddress;
    int ipPort;
    
    AsyncMqttClient mqtt;
    bool isRetain;
    String lwtTopic;

public:
    bool publish(const char* topic, uint8_t qos, bool retain, const char* payload = nullptr, size_t length = 0);
    bool publishStat(const char* name, uint8_t qos, bool retain, const char* payload = nullptr, size_t length = 0);
    bool publishTele(const char* name, uint8_t qos, bool retain, const char* payload = nullptr, size_t length = 0);

};

#endif
#endif