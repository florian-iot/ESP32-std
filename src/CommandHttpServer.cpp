#include <ESPAsyncWebServer.h>
#include "CommandHttpServer.h"
#include "UEvent.h"



CommandHttpServer::CommandHttpServer()
{
}

CommandHttpServer::~CommandHttpServer()
{
}

/**
 * URL format:
 * /cmd?c=<command>+<arg>+<arg>...
 */
void CommandHttpServer::init(AsyncWebServer *server, CommandMgr *commandMgr)
{
  this->commandMgr = commandMgr;
  server->on("/cmd", [this](AsyncWebServerRequest *request) {
    AsyncWebParameter *p = request->getParam("c");
    if (p == nullptr) {
      request->send(404, "text/plain", "Expecting parameter \"c\" with the command line\n");
    } else {
      String msg;
      msg = p->value();
      bool processed = this->commandMgr->processCommandLine("HTTP", &msg);
      if (processed) {
        request->send(200, "text/plain", msg);
      } else {
        char r[512];
        snprintf(r, sizeof(r), "Message not processed: %s", msg.c_str());
        r[sizeof(r) - 1] = '\0';
        request->send(500, "text/plain", msg);
      }
    }
  });
}
