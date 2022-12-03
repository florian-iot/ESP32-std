#include <CompilationOpts.h>

#ifdef USE_MQTT

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "MqttService.h"
#include "LogMgr.h"

/*****
 * <topic> - by default the wifi hostname
 * optionally a <prefix>
 * 
 * /<prefix>/tele/<topic>/LWT xxx -- last will and testament
 * /<prefix>/tele/<topic>/<measure/> xxx -- telemetry at specified interval
 * /<prefix>/cmnd/<topic>/<command> xxx -- incoming command
 * /<prefix>/stat/<topic>/<command> xxx -- reply for command or status changes
 */

void MqttService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, WifiAsyncService *wifi, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("mqtt");
    this->wifi = wifi;

    ServiceCommands *cmd = commandMgr->getServiceCommands("mqtt");
    initCommands(cmd);

    // defaults
    ipAddress = "";
    ipPort = 1883;
    topic = wifi->getHostName();
    prefix = "";
    isRetain = false;

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

    // strings that must be available as long as mqtt is connected
    lwtTopic = prefix + "tele/" + topic + "/LWT";

    mqtt.setClientId(topic.c_str())
        .setServer(ipAddress.c_str(), ipPort)
        .setCleanSession(true)
        .setWill(lwtTopic.c_str(), 1, true, "offline");
    
    mqtt.onConnect([this](bool isSessionPresent) {
        mqtt.publish(lwtTopic.c_str(), 1, true, "online");
    });
    mqtt.onDisconnect([this](AsyncMqttClientDisconnectReason reason) {
Serial.println("MQTT onDisconnect()");
    });

    wifi->onStart([this](bool isStandalone) {
        if (isEnabled) {
Serial.println("MQTT connecting on wifi start");
            mqtt.connect();
        }
    });
    if (isEnabled && (wifi->getStatus() == WifiAsyncService::CONNECTED || wifi->getStatus() == WifiAsyncService::STANDALONE)) {
Serial.println("MQTT connecting on init, as wifi is already started");
        mqtt.connect();
    }

    wifi->onStop([this]() {
        if (isEnabled) {
Serial.println("MQTT disconnecting on wifi stop");
            mqtt.publish(lwtTopic.c_str(), 0, true, "offline");
            mqtt.disconnect();
        }
    });

}

bool MqttService::publish(const char *topic, uint8_t qos, bool retain, const char* payload, size_t length)
{
    int rc = mqtt.publish(topic, qos, retain, payload, length);
    return rc != 0;
}

bool MqttService::publishStat(const char* name, uint8_t qos, bool retain, const char* payload, size_t length)
{
    String t(prefix + "stat/" + topic + "/" + name);
    int rc = mqtt.publish(t.c_str(), qos, retain, payload);
    return rc != 0;
}

bool MqttService::publishTele(const char* name, uint8_t qos, bool retain, const char* payload, size_t length)
{
    String t(prefix + "tele/" + topic + "/" + name);
    int rc = mqtt.publish(t.c_str(), qos, retain, payload);
    return rc != 0;
}

void MqttService::initCommands(ServiceCommands *cmd)
{
    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("enabled", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .helpOn("--> Enable the mqtt service")
        .helpOff("--> Disable the mqtt service")
        .ptr(&isEnabled)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (isLoading) {
                isEnabled = val;
                return true;
            }
            if (isEnabled == val) {
                *msg = "Already "; msg->concat(isEnabled ? "enabled" : "disabled");
                return true;
            }
            if (val) {
                isEnabled = true;
                if (wifi->getStatus() == WifiAsyncService::CONNECTED || wifi->getStatus() == WifiAsyncService::STANDALONE) {
                    mqtt.connect();
                    *msg = "MQTT connection initiated";
                } else {
                    // else connection will be initiated when wifi gets connected
                    *msg = "MQTT connection will be initiated when wifi is connected";
                }
            } else {
                mqtt.publish(lwtTopic.c_str(), 1, true, "offline");
                mqtt.disconnect();
                *msg = "MQTT disconnected";
                isEnabled = false;
            }
            return true;            
        })
    );
    cmd->registerStringData(ServiceCommands::StringDataBuilder("ipAddress", true)
        .cmd("ipAddress")
        .help("--> IP address of the MQTT server -- needs disable/enable")
        .ptr(&ipAddress)
    );
    cmd->registerIntData(ServiceCommands::IntDataBuilder("ipPort", true)
        .cmd("ipPort")
        .help("--> IP port of the MQTT server, default is 1883 -- needs disable/enable")
        .ptr(&ipPort)
    );
    cmd->registerStringData(ServiceCommands::StringDataBuilder("topic", true)
        .cmd("topic")
        .help("--> Part of the MQTT topic for this device, defaults to wifi host name")
        .ptr(&topic)
        .setFn([this](const String &val, bool isLoading, String *msg) {
            if (!isLoading && mqtt.connected()) {
                mqtt.publish(lwtTopic.c_str(), 0, false, nullptr);
            }
            lwtTopic = prefix + "tele/" + topic + "/LWT";
            if (!isLoading) {
                mqtt.setWill(lwtTopic.c_str(), 1, true, "offline");
                if (mqtt.connected()) {
                    mqtt.publish(lwtTopic.c_str(), 1, true, "online");
                }
            }
            return true;
        })
    );
    cmd->registerStringData(ServiceCommands::StringDataBuilder("prefix", true)
        .cmd("prefix")
        .help("--> Prefix for the MQTT topic for this device, set to \"off\" to not have a prefix")
        .getFn([this](String *val) {
            if (prefix == "") {
                *val = "off";
            } else {
                *val = prefix;
            }
        })
        .setFn([this](const String &val, bool isLoading, String *msg) {
            if (!isLoading && mqtt.connected()) {
                mqtt.publish(lwtTopic.c_str(), 0, false, nullptr);
            }

            if (val == "off") {
                prefix = "";                   
            } else {
                if (val.endsWith("/")) {
                    prefix = val;
                } else {
                    prefix = val; prefix.concat("/");
                }
            }

            lwtTopic = prefix + "tele/" + topic + "/LWT";
            if (!isLoading) {
                mqtt.setWill(lwtTopic.c_str(), 1, true, "offline");
                if (mqtt.connected()) {
                    mqtt.publish(lwtTopic.c_str(), 1, true, "online");
                }
            }
            if (prefix.isEmpty()) {
                *msg = "Prefix cleared";
            } else {
                *msg = "Prefix set to " + prefix;
            }
            return true;
        })
    );

    cmd->registerBoolData(ServiceCommands::BoolDataBuilder("setRetain", true)
        .cmd("setRetain")
        .help("--> Sets the retain flag used by \"publish\" command")
        .ptr(&isRetain)
    );

    cmd->registerStringData(ServiceCommands::StringDataBuilder("publish", true)
        .cmd("publish")
        .help("publish <topic> <message> --> publish the message in topic, retain flag is set with \"setRetain\" command")
        .isPersistent(false)
        .setFn([this](const String &val, bool isLoading, String *msg) {
            String t, m;
            int i = val.indexOf(' ');
            if (i >= 0) {
                t = val.substring(0, i);
                m = val.substring(i);
                m.trim();
            } else {
                t = val;
                m.clear();
            }
            uint32_t rc = mqtt.publish(t.c_str(), 0, isRetain, m.c_str());
            *msg = "Publishing \"" + t + "\" \"" + m + "\", return code is " + rc;
            return true;
        })
    );

}

#endif
