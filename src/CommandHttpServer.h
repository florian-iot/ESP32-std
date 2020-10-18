#ifndef COMMAND_HTTP_SERVER_H
#define COMMAND_HTTP_SERVER_H

#include <ESPAsyncWebServer.h>
#include "UEvent.h"
#include "Monitor.h"
#include "CommandMgr.h"

class CommandHttpServer {
public:
  CommandHttpServer();
  ~CommandHttpServer();
  void init(AsyncWebServer *server, CommandMgr *commandMgr);
private:
  CommandMgr *commandMgr;

  void processCommand(String cmdm, AsyncWebSocketClient *client);

  void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);

};

#endif
