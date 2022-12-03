#include <CompilationOpts.h>

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Util.h"
#include <MDNS.h>
#include <ESPmDNS.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiMulti.h>
#include <AsyncTCP.h>
#include <DNSServer.h>
#include <esp_task_wdt.h>

// Use from 0 to 4. Higher number, more debugging messages and memory usage.
#define _ESPASYNC_WIFIMGR_LOGLEVEL_    0
#define USE_ESP_WIFIMANAGER_NTP     false
#include <ESPAsync_WiFiManager.h>

#include <WifiServiceAsync.h>

void WifiAsyncService::init(AsyncWebServer *webServer, UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->webServer = new AsyncWebServer(80);
    this->logger = logMgr->newLogger("wifi");

    ServiceCommands *cmd = commandMgr->getServiceCommands("wifi");
    initCommands(cmd);

    // defaults
    isAutoStart = true;
    isAPOnly = false;
    String mac = WiFi.macAddress();
    apIp = IPAddress(172, 217, 28, 1);
    apPassword = AUTOCONNECT_PSK;
    hostName = "esp32-" + mac.substring(mac.length() - 5, mac.length() - 3) + mac.substring(mac.length() - 2);

    wifiCredsFileName = "/wifiCredentials.conf.json";

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
    wifiInternalInitializedEventType = eventLoop->getEventType("wifi-internal", "wifiInitialized");
    wifiInitializedEventType = eventLoop->getEventType("wifi", "initialized");
    isStarted = false;
    status = WifiStatus::STOPPED;

    wifiInternalInitializedEventHandler = UEventHandle_t();
    wifiCheckTimer.init(eventLoop, [this](UEventLoopTimer *timer) {
        if (dfa.getState() == WIFI_DONE && WiFi.status() != WL_CONNECTED && status == WifiStatus::CONNECTED) {
            status = WifiStatus::RECONNECTING;
            dfa.handleInput(WIFI_START);
        }
    });

    // init hardware

    // start DFA
    initDfa();

    if (isAutoStart) {
        dfa.queueInput(WIFI_START, 1);
    }
}

void WifiAsyncService::initDfa()
{
    dfa.init(eventLoop, logger, WIFI_INITIAL);

    dfa.onInput([this](Dfa *dfa, Dfa::State state, Dfa::Input input) {

        if (state.is(WIFI_INITIAL)) {
            if (input.is(WIFI_START)) {
                return dfa->transitionTo(WIFI_STARTUP);
            } else {
                return dfa->transitionError();
            }
        } else if (state.is(WIFI_STARTUP)) {
            if (input.is(Dfa::Input::ENTER_STATE)) {
                status = WifiStatus::STARTING;

                // initialize WIFI in a separate thread, on completion go to next state

                // we can't call dfa->handleInput() in another thread, it must be done within
                // the event loop. So we use an event (which can be sent from another thread)
                if (!wifiInternalInitializedEventHandler.isSet()) {
                    wifiInternalInitializedEventHandler = eventLoop->onEvent(wifiInternalInitializedEventType, [this](UEvent *event) {
                        this->dfa.handleInput(WIFI_INITIALIZED);
                        return true;
                    });
                }
                Util::ThreadOptions opt;
                opt.stackSize = 12288 * 2;
                Util::runAsThread("wifiInitializer", opt, [this]() {
                    initWifi();
                    eventLoop->queueEvent(UEvent(wifiInternalInitializedEventType, (int64_t)0), nullptr, nullptr);
                });
                return dfa->noTransition();
            } else if (input.is(WIFI_INITIALIZED)) {
                if (WiFi.getMode() == WIFI_AP) {
                    status = WifiStatus::STANDALONE;
                } else {
                    status = WifiStatus::CONNECTED;
                }
                eventLoop->queueEvent(UEvent(wifiInitializedEventType, status), nullptr, nullptr);
                for (int i = 0; i < onStartList.size(); i++) {
                    onStartList.get(i)(status == STANDALONE ? true : false);
                }
                return dfa->transitionTo(WIFI_DONE);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(WIFI_DONE)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                wifiCheckTimer.setInterval(1000);
                return dfa->noTransition();
            } else if (input.is(WIFI_START)) {
                wifiCheckTimer.cancelInterval();
                if (status == WifiStatus::STOPPED) {
                    status = WifiStatus::STARTING;
                } else {
                    status = WifiStatus::RECONNECTING;
                }
                eventLoop->queueEvent(UEvent(wifiInitializedEventType, status), nullptr, nullptr);
                return dfa->transitionTo(WIFI_STARTUP);
            } else if (input.is(WIFI_STOP)) {
                wifiCheckTimer.cancelInterval();
                status = WifiStatus::STOPPING;
                eventLoop->queueEvent(UEvent(wifiInitializedEventType, status), nullptr, nullptr);
                WiFi.disconnect(true, false);
                status = WifiStatus::STOPPED;
                eventLoop->queueEvent(UEvent(wifiInitializedEventType, status), nullptr, nullptr);
                for (int i = 0; i < onStopList.size(); i++) {
                    onStopList.get(i)();
                }
                return dfa->noTransition();
            } else {
                return dfa->noTransition();
            }

        } else {
            return dfa->transitionError();
        }

    });
}

void WifiAsyncService::initCommands(ServiceCommands *cmd)
{
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("autoStart", true)
        .cmd("autoStart")
        .help("--> If set to true, wifi is started automatically.")
        .ptr(&isAutoStart)
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("apOnly", true)
        .cmd("apOnly")
        .help("--> If set to true, wifi is started only as Access Point, without connecting to another wifi network.")
        .ptr(&isAPOnly)
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("apIp", true)
        .cmd("apIp")
        .help("--> IP address in standalone mode -- requires restart")
        .setFn([this](const String &val, bool isLoading, String *msg) {
            IPAddress addr;
            if (addr.fromString(val)) {
                apIp = addr;
                *msg = "Standalone IP address set to ";
                *msg += apIp.toString();
            } else {
                *msg = "Couldn't parse \"" + val + "\" as an IP address";
            }
            return true;
        })
        .getFn([this](String *val) {
            *val = apIp.toString();
        })
    );


    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("apPassword", true)
        .cmd("apPassword")
        .help("--> Password for standalone mode")
        .ptr(&apPassword)
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("hostname", true)
        .cmd("hostname")
        .help("--> Host name, as well as the SSID in standalone mode")
        .ptr(&hostName)
    );


    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("status", true)
        .cmd("status")
        .help("--> Current Wifi status")
        .isPersistent(false)
        .getFn([this](String *val) {
            switch (status) {
                case WifiStatus::CONNECTED: *val = "Connected"; break;
                case WifiStatus::RECONNECTING: *val = "Reconnecting"; break;
                case WifiStatus::STANDALONE: *val = "Standalone"; break;
                case WifiStatus::STARTING: *val = "Starting"; break;
                case WifiStatus::STOPPED: *val = "Stopped"; break;
                case WifiStatus::STOPPING: *val = "Stopping"; break;
            }
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("ip", true)
        .cmd("ip")
        .help("--> Current IP address")
        .isPersistent(false)
        .getFn([this](String *val) {
            if (status == WifiStatus::CONNECTED) {
                *val = WiFi.localIP().toString();
            } else if (status == WifiStatus::STANDALONE) {
                *val = WiFi.softAPIP().toString();
            } else if (WiFi.softAPIP() != IPAddress(0, 0, 0, 0)) {
                *val = WiFi.softAPIP().toString();
            } else {
                *val = "0.0.0.0";
            }
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("SSID", true)
        .cmd("SSID")
        .help("--> Current SSID")
        .isPersistent(false)
        .getFn([this](String *val) {
            if (status == WifiStatus::CONNECTED) {
                *val = WiFi.SSID();
            } else if (status == WifiStatus::STANDALONE) {
                *val = WiFi.softAPSSID();
            } else if (WiFi.softAPIP() != IPAddress(0, 0, 0, 0)) {
                *val = WiFi.softAPSSID();
            } else {
                *val = "[none]";
            }
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("start", true)
        .cmdOn("start")
        .helpOn("--> Starts the wifi, if not started yet.")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (status != STOPPED) {
                *msg = "Wifi already started";
            } else {
                startWifi();
                *msg = "Wifi starting";
            }
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("stop", true)
        .cmdOn("stop")
        .helpOn("--> Stops the wifi, if already started.")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (status != WifiStatus::CONNECTED && status != WifiStatus::STANDALONE) {
                *msg = "Wifi is not connected/standalone";
            } else {
                stopWifi();
                *msg = "Wifi stopping";
            }
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("forget", true)
        .cmdOn("forget")
        .helpOn("--> Disconnects and forgets the last connected network, so that it won't be connected automatically.")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            if (status != WifiStatus::CONNECTED) {
                *msg = "Wifi is not connected";
            } else {
                WiFi.disconnect(true, true);
                *msg = "Disconnected and current network is forgotten";
            }
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("showStoredCredentials", true)
        .cmdOn("showStoredCredentials")
        .helpOn("--> Shows stored wifi credentials")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            DynamicJsonBuffer buf;
            JsonObject *creds = nullptr;
            // read current list of credentials
            if (!SPIFFS.exists(wifiCredsFileName)) {
                *msg = String("Credentials file ") + wifiCredsFileName + " does not exist";
                return true;
            }
            File f = SPIFFS.open(wifiCredsFileName, "r");
            creds = &buf.parseObject(f);
            f.close();
            if (!creds->success()) {
                *msg = String("Error reading JSON data from credentials file ") + wifiCredsFileName;
                return true;
            }
            for (auto it = creds->begin(); it != creds->end(); ++it) {
                const char *ssid = it->key;
                const char *pass = it->value.as<const char *>();
                if (ssid != nullptr && strlen(ssid) > 0) {
                    msg->concat("    ");
                    msg->concat(ssid);
                    msg->concat(":");
                    msg->concat(pass);
                }
            }
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("clearStoredCredentials", true)
        .cmdOn("clearStoredCredentials")
        .helpOn("--> Clear all stored wifi credentials")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            const char *configFileName = "wifiCredentials.conf.json";
            // read current list of credentials
            if (!SPIFFS.exists(configFileName)) {
                *msg = String("Credentials file ") + configFileName + " does not exist";
                return true;
            }
            File f = SPIFFS.open(configFileName, "w");
            f.println("{ }");
            f.close();
            *msg = "Stored wifi credentials cleared";
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("asyncTcpStackHighWaterMark", true)
        .cmd("asyncTcpStackHighWaterMark")
        .help("--> Async TCP AsyncClient::getStackHighWaterMark()")
        .isPersistent(false)
        .getFn([this]() {
return -1; //            return AsyncClient::getStackHighWaterMark();
        })
    );

}

void WifiAsyncService::initWifi()
{
    bool isConnected = false;

Serial.printf("Initializing wifi, heap %u (%u free), stack high watermark %u\n",
    ESP.getHeapSize(), ESP.getFreeHeap(), uxTaskGetStackHighWaterMark(nullptr));

    if (isAPOnly) {
        WiFi.mode(WIFI_AP);
        WiFi.softAPConfig(apIp, apIp, IPAddress(255, 255, 255, 0));
        WiFi.softAPsetHostname(hostName.c_str());
        WiFi.persistent(false);
        bool isConnected = WiFi.softAP(hostName.c_str(), apPassword.c_str());

        if (!isConnected) {
            logger->error("Couldn't start wifi in AP mode, error in call to WiFi.softAP()");
        } else {
            logger->info("Started wifi in AP mode with IP {}, password {}",
                WiFi.softAPIP().toString().c_str(),
                apPassword.c_str());
        }
    } else {
        // try to connect with credentials of last connection, if any
        WiFi.mode(WIFI_STA);
        logger->debug("Trying to connect to last known network");
        WiFi.begin();
        uint8_t status = WiFi.waitForConnectResult();
        if (status == WL_CONNECTED) {
            logger->debug("Started wifi in STA mode");
            logger->debug("Started wifi in STA mode at SSID {}, IP {}",
                WiFi.SSID().c_str(),
                WiFi.localIP().toString().c_str());
            isConnected = true;
        } else {
            logger->debug("Failed to connect to last known network, will try known networks");
        }

        DynamicJsonBuffer buf;
        JsonObject *creds = nullptr;
        if (!isConnected) {
            // read current list of credentials
            if (SPIFFS.exists(wifiCredsFileName)) {
                File f = SPIFFS.open(wifiCredsFileName, "r");
                creds = &buf.parseObject(f);
                f.close();
                if (!creds->success()) {
                    logger->error("Error reading wifi credentials as JSON data from file {}, file ignored",
                        wifiCredsFileName);
                    creds = nullptr;
                }
            } else {
                logger->info("No existing wifi credentials, file {} not found", wifiCredsFileName);
            }
        }

        // if we didn't connect with last credentials, use WiFiMulti to try all known credentials
        if (!isConnected && creds != nullptr && creds->size() > 0) {
            WiFiMulti wifiMulti;
            logger->debug("Trying to connect to known networks ({} networks)", creds->size());
            for (JsonObject::iterator it = creds->begin(); it != creds->end(); ++it) {
                const char *ssid = it->key;
                const char *pass = it->value.as<const char *>();
                if (ssid != nullptr && strlen(ssid) > 0) {
                    logger->debug("    {}", ssid);
                    wifiMulti.addAP(ssid, (pass != nullptr && strlen(pass) != 0 ? pass : nullptr));
                }
            }
            uint8_t status = wifiMulti.run(10000);
            if (status == WL_CONNECTED) {
                logger->debug("Started wifi in STA mode using best known network at SSID {}, IP {}",
                    WiFi.SSID().c_str(),
                    WiFi.localIP().toString().c_str());
                isConnected = true;
            } else {
                logger->debug("Couldn't connect to any of the known networks, will start wifi manager");
            }
        }

        esp_task_wdt_reset();

        // if we didn't manage to connect with known credentials, start up Wifi Manager's capturing portal
        if (!isConnected) {
            AsyncWebServer webServer(80);
            DNSServer dnsServer;
            ESPAsync_WiFiManager wifiManager(&webServer, &dnsServer, hostName.c_str());

            logger->info("Starting config portal at SSID {}, IP {}, port 80, wifi password: {}",
                hostName.c_str(),
                apIp.toString().c_str(),
                apPassword.c_str());
            WiFi.mode(WIFI_MODE_APSTA);
            WiFi.softAPConfig(apIp, apIp, IPAddress(255, 255, 255, 0));
            WiFi.softAPsetHostname(hostName.c_str());
            WiFi.softAP(hostName.c_str(), apPassword.c_str());

            wifiManager.setDebugOutput(true);
            wifiManager.setAPStaticIPConfig(apIp, apIp, IPAddress(255, 255, 255, 0));
            logger->debug("Starting configuration portal");
            isConnected = wifiManager.startConfigPortal(hostName.c_str(), apPassword.c_str());
            logger->debug("Configuration portal returned, connected: {}",
                isConnected ? "true" : "false");

            // see if we've new credentials, save them to config file
            if (creds == nullptr) {
                creds = &buf.createObject();
            }
            bool credsChanged = false;
            String s = wifiManager.getSSID(0);
            String p = wifiManager.getPW(0);
            if (s != nullptr && s != "") {
                (*creds)[s] = p;
                credsChanged = true;
            }
            s = wifiManager.getSSID(1);
            p = wifiManager.getPW(1);
            if (s != nullptr && s != "" && !creds->containsKey(s)) {
                (*creds)[s] = p;
                credsChanged = true;
            }
            // keep no more than 10 entries
            while (credsChanged && creds->size() > 10) {
                auto it = creds->begin();
                creds->remove(it->key);
            }
            // if creds changed, save
            if (credsChanged) {
                File f = SPIFFS.open(wifiCredsFileName, "w");
                if (f) {
                    creds->prettyPrintTo(f);
                    f.close();
                    logger->info("Wifi credentials saved");
                } else {
                    logger->error("Couldn't write wifi credentials file {}, changes are lost",
                        wifiCredsFileName);
                }
            }

            creds = nullptr;
            buf.clear();
        }
    }
}

const char *WifiAsyncService::getHostName()
{
    return hostName.c_str();
}

WifiAsyncService::WifiStatus WifiAsyncService::getStatus()
{
    return status;
}

bool WifiAsyncService::startWifi()
{
    if (status != WifiStatus::STOPPED) {
        return false;
    }
    dfa.queueInput(WIFI_START, 1); // does it stack overflow if dfa.handleInput()?
    return true;
}

bool WifiAsyncService::stopWifi()
{
    if (status == WifiStatus::STOPPED) {
        return false;
    }
    dfa.handleInput(WIFI_STOP);
    return true;
}

int WifiAsyncService::onStart(std::function<void(bool isStandalone)> callback)
{
    return onStartList.add(callback);
}

void WifiAsyncService::removeOnStart(int id) {
    onStartList.remove(id);
}

int WifiAsyncService::onStop(std::function<void()> callback)
{
    return onStopList.add(callback);
}

void WifiAsyncService::removeOnStop(int id) {
    onStopList.remove(id);
}
