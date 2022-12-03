#ifndef COMMAND_WEBSOCKETS_H
#define COMMAND_WEBSOCKETS_H

#include <ESPAsyncWebServer.h>
#include "UEvent.h"
#include "Monitor.h"
#include "CommandMgr.h"
#include "SystemService.h"
#include "RebootDetectorService.h"
#include "LogMgr.h"

class WebSocketsServer {
public:
  WebSocketsServer();
  ~WebSocketsServer();
  void init(AsyncWebServer *server, CommandMgr *commandMgr, SystemService *system, RebootDetectorService *rebootDetector, LogMgr *logMgr);
  void cleanupClients();

private:
  AsyncWebSocket ws;
  // AsyncEventSource events;
  CommandMgr *commandMgr;
  SystemService *system;
  RebootDetectorService *rebootDetector;
  Logger *logger;

  void processCommand(String cmdm, AsyncWebSocketClient *client);

  void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);

};

#endif
