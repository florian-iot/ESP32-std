#include <ESPAsyncWebServer.h>
#include "CommandWebSockets.h"
#include "UEvent.h"
#include "RebootDetectorService.h"
#include "Util.h"

WebSocketsServer::WebSocketsServer()
    : ws("/ws")
{
}

WebSocketsServer::~WebSocketsServer()
{
}

void WebSocketsServer::init(AsyncWebServer *server, CommandMgr *commandMgr, SystemService *system,
                            RebootDetectorService *rebootDetector, LogMgr *logMgr)
{
  this->commandMgr = commandMgr;
  this->system = system;
  this->rebootDetector = rebootDetector;
  this->logger = logMgr->newLogger("WebSockets");
  ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
             { onWsEvent(server, client, type, arg, data, len); });
  server->addHandler(&ws);

  // events.onConnect([](AsyncEventSourceClient *client) {
  //   client->send("hello!", NULL, millis(), 1000);
  // });
  // server->addHandler(&events);
}

void WebSocketsServer::cleanupClients()
{
 ws.cleanupClients();
}

void WebSocketsServer::onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                                 AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  // Serial.printf("WebSocketsServer: Called onWsEvent, type = %d\n", type);
  if (type == WS_EVT_CONNECT)
  {
  // Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
    String uptimeStr;
    Util::durationToStr(&uptimeStr, system->uptime());
    client->printf("Welcome to esp32, client %u! :) Uptime %s, current reboot level: %s ",
                   client->id(), uptimeStr.c_str(), rebootDetector->isActive() ? rebootDetector->getRebootLevelStr() : "N/A");
    //    client->ping(); --> crashes after receipt of pong when browser is from iOS
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    logger->debug("Client {} disconnect", client->id());
  }
  else if (type == WS_EVT_ERROR)
  {
    logger->debug("Client {} error({}): {}", client->id(), *((uint16_t *)arg), (char *)data);
  }
  else if (type == WS_EVT_PONG)
  {
    // logger->trace("Client {} pong[{}]: {}", client->id(), len, (len > 0) ? (char *)data : "");
    logger->trace("Client {} pong[{}]", client->id(), len);
  }
  else if (type == WS_EVT_DATA)
  {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    // Serial.printf("WebSocketsServer: Called onWsEvent, type = WS_EVT_DATA, opcode: %s, info->final: %d, info->index: %llu, info->len: %llu\n",
    //               (info->opcode == WS_TEXT ? "text" : "binary"), info->final, info->index, info->len);
    String msg = "";
    if (info->final && info->index == 0 && info->len == len)
    {
      // the whole message is in a single frame and we got all of it's data
      // Serial.printf("ws[%s][%u] %s-message[%llu]\n", server->url(), client->id(),
      //               (info->opcode == WS_TEXT) ? "text" : "binary", info->len);

      if (info->opcode == WS_TEXT)
      {
        for (size_t i = 0; i < info->len; i++)
        {
          msg += (char)data[i];
        }
      }
      else
      {
        char buff[4];
        for (size_t i = 0; i < info->len; i++)
        {
          sprintf(buff, "%02x ", (uint8_t)data[i]);
          msg += buff;
        }
      }

      if (info->opcode == WS_TEXT)
      {
        if (msg.equals("noop"))
        {
          // ignore
        }
        else
        {
          logger->debug("Client {} received command \"{}\"", client->id(), msg.c_str());
          char buf[256];
          strcpy(buf, "Command: ");
          strncat(buf, msg.c_str(), sizeof(buf) - strlen(buf));
          buf[sizeof(buf) - 1] = '\0';
          client->text(buf);

          processCommand(msg, client);
        }
      }
      else
      {
        client->binary("I got your binary message");
      }
    }
    else
    {
      // message is comprised of multiple frames or the frame is split into multiple packets
      if (info->index == 0)
      {
        if (info->num == 0)
        {
          logger->debug("Client {} {} message start", client->id(),
                        (info->message_opcode == WS_TEXT) ? "text" : "binary");
        }
        logger->debug("Client {} frame {} start {}", client->id(), info->num, info->len);
      }

      logger->debug("Client {} frame {} {}[{} - {}]", client->id(), info->num,
                    (info->message_opcode == WS_TEXT) ? "text" : "binary", info->index, info->index + len);

      if (info->opcode == WS_TEXT)
      {
        for (size_t i = 0; i < info->len; i++)
        {
          msg += (char)data[i];
        }
      }
      else
      {
        char buff[4];
        for (size_t i = 0; i < info->len; i++)
        {
          sprintf(buff, "%02x ", (uint8_t)data[i]);
          msg += buff;
        }
      }

      if ((info->index + len) == info->len)
      {
        logger->debug("Client {} frame {} end {}", client->id(), info->num, info->len);
        if (info->final)
        {
          logger->debug("Client {} {} message end", client->id(),
                        (info->message_opcode == WS_TEXT) ? "text" : "binary");
          if (info->message_opcode == WS_TEXT)
          {
            logger->debug("Client {} received command \"{}\"", client->id(), msg.c_str());
            char r[512];
            snprintf(r, sizeof(r), "> %s", msg.c_str());
            r[sizeof(r) - 1] = '\0';
            client->text(r);

            processCommand(msg, client);
          }
          else
          {
            client->binary("I got your binary message");
          }
        }
      }
    }
  }
}

void WebSocketsServer::processCommand(String msg, AsyncWebSocketClient *client)
{
//  client->text(":)\n");

  logger->trace("Client {} processing command: {}", client->id(), msg.c_str());
  bool processed = commandMgr->processCommandLine("WS", &msg);
  if (processed) {
    logger->trace("Client {} command processed: {}", client->id(), msg.c_str());
    client->text(msg.c_str());
  } else {
    logger->trace("Client {} command not processed: {}", client->id(), msg.c_str());
    char r[512];
    snprintf(r, sizeof(r), "Message not processed: %s", msg.c_str());
    r[sizeof(r) - 1] = '\0';
    client->text(r);
  }
}
