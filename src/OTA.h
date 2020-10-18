#ifndef OTA_H
#define OTA_H

#include <ESPAsyncWebServer.h>
#include "LogMgr.h"

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
    bool (*_onUpdateStart)();
    void (*_onUpdateEnd)(bool succeeded);

    void updateError(AsyncWebServerRequest *request, PGM_P msg, ...);

public:
    OTA();
    void init(AsyncWebServer *server, LogMgr *logMgr, bool (*onUpdateStart)(), void (*onUpdateEnd)(bool succeeded));

};


#endif
