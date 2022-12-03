#include <CompilationOpts.h>

#ifndef INCL_WIFI_ASYNC_H
#define INCL_WIFI_ASYNC_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "Util.h"
#include <DNSServer.h>

class ESPAsync_WiFiManager;

/**
 * Events:
 *     Upon connection termination (successful or not, or not connected, if so parametered): wifi:initialized
 *         data: 0 for not conneced, 1 for STA, 2 for AP
 * TODO See what can be done for disconnections, reconnections etc.
 *
 * Connection process
 *
 * If isAPOnly is set, the AP mode is entered, the portal is set up and no
 * other network connection is tried.
 *
 * Else, an attempt is made to connect to last connected network.
 * If not successful, all other known networks are tried.
 * If still not successful, AP mode is entered and the portal is set up. The parameters
 * can be set via a browser connected to the AP network. If existing stored
 * credentials exist for networks that haven't been checked for connection yet,
 * every 5 minutes a network scan will be performed to see if such networks have
 * come to existence, and connection is tried on them. Every 30 minutes the list
 * of connections that have been checked is cleared (so they're checked again).
 *
 * If already connected in STA mode and the connection is lost, connection is retried to the
 * last connected network for 2 minutes, then AP mode is set and a network scan is retried
 * every 5 minutes.
 *
 * If the autoconnect flag is set, the connection process starts at the initialization, else
 * it is started upon button press (if the button is configured).
 *
 * A button press while in AP mode causes the current list of tried networks to be forgotten
 * and a new network scan to be started, so all known networks will be tried for connection.
 *
 * Events:
 *     wifi:starting
 *     wifi:initialized (either AP or STA, as defined in config)
 *     wifi:ap
 *     wifi:sta
 *     wifi:reconnecting
 *
 **/
class WifiAsyncService {
public:
    void init(AsyncWebServer *webServer, UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr);

    const char *getHostName();
    // false if already started
    bool startWifi();
    bool stopWifi();
    enum WifiStatus {
        STOPPED,
        STARTING,
        STANDALONE,
        CONNECTED,
        RECONNECTING,
        STOPPING
    };
    WifiStatus getStatus();

    int onStart(std::function<void(bool isStandalone)> callback);
    void removeOnStart(int id);
    int onStop(std::function<void()> callback);
    void removeOnStop(int id);

private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;
    AsyncWebServer *webServer;

    // If true, the connection process will start upon initialization. Else,
    // via call (e.g., when button is pressed)
    bool isAutoStart;
    bool isAPOnly; // if false, will scan periodically and try to connect in STA mode
    IPAddress apIp;
    String apPassword;
    String hostName;

    const char *wifiCredsFileName;
    bool isStarted; // true if start was requested, be that automatic or manually
    WifiStatus status;
    UEventLoopTimer wifiCheckTimer;
    CallbackList<void (bool)> onStartList;
    CallbackList<void ()> onStopList;

    uint32_t wifiInternalInitializedEventType;
    UEventHandle_t wifiInternalInitializedEventHandler;
    uint32_t wifiInitializedEventType;

    Dfa dfa;

    Dfa::Input WIFI_START = dfa.nextInput("WIFI_START");
    Dfa::Input WIFI_STOP = dfa.nextInput("WIFI_STOP");

    Dfa::State WIFI_INITIAL = dfa.nextState("WIFI_INITIAL");
    Dfa::State WIFI_STARTUP = dfa.nextState("WIFI_STARTUP");
    Dfa::Input WIFI_INITIALIZED = dfa.nextInput("WIFI_INITIALIZED");
    Dfa::State WIFI_DONE = dfa.nextState("WIFI_DONE");

    void initCommands(ServiceCommands *cmd);
    void initDfa();
    void initWifi();
};

#endif