#include <WiFi.h>
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
void CommandHttpServer::init(AsyncWebServer *server, CommandMgr *commandMgr, LogMgr *logMgr)
{
  this->commandMgr = commandMgr;
  this->logger = logMgr->newLogger("CommandHttpServer");
  server->on("/cmd", [this](AsyncWebServerRequest *request) {
    AsyncWebParameter *p = request->getParam("c");
    if (p == nullptr) {
      request->send(404, "text/plain", "Expecting parameter \"c\" with the command line\n");
    } else {
      String msg;
      msg = p->value();

      // AsyncResponseStream *rs = request->beginResponseStream("text/plain");
      // bool processed = this->commandMgr->processCommandLine("HTTP", &msg, [rs](const String &responseMsg) {
      //   rs->setContentType("text/plain");
      //   rs->setContentLength(responseMsg.length());
      //   rs->write(response);
      //   rs->close();
      // });

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
