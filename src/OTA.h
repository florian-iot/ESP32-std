#ifndef OTA_H
#define OTA_H

#include <ESPAsyncWebServer.h>
#include "LogMgr.h"
#include "Util.h"

class OTA {
private:
    AsyncWebServer *_server;
    LogMgr *logMgr;
    Logger *logger;

    /** Request that is performing an upload, null if none */
    AsyncWebServerRequest *_uploadingRequest;
    bool _updateSucceeded;
    int _uploadCount;
    bool _hasUpdateError;

    CallbackList<bool() > otaStartCallbacks;
    CallbackList<void(bool success)> otaEndCallbacks;

    void updateError(AsyncWebServerRequest *request, PGM_P msg, ...);

public:
    OTA();
    void init(AsyncWebServer *server, LogMgr *logMgr, bool (*onUpdateStart)(), void (*onUpdateEnd)(bool succeeded));
    int onStart(std::function<bool()> fn);
    void removeOnStart(int handle);
    int onEnd(std::function<void(bool success)> fn);
    void removeOnEnd(int handle);
};


#endif
