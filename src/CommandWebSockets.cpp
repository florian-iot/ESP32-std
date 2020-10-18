#include <ESPAsyncWebServer.h>
#include "CommandWebSockets.h"
#include "UEvent.h"

WebSocketsServer::WebSocketsServer()
    : ws("/ws")
{
}

WebSocketsServer::~WebSocketsServer()
{
}

void WebSocketsServer::init(AsyncWebServer *server, CommandMgr *commandMgr)
{
  this->commandMgr = commandMgr;
  ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    onWsEvent(server, client, type, arg, data, len);
  });
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
  if (type == WS_EVT_CONNECT)
  {
    Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
    client->printf("Welcome to esp32, client %u! :)", client->id());
    client->ping();
  }
  else if (type == WS_EVT_DISCONNECT)
  {
    Serial.printf("ws[%s][%u] disconnect\n", server->url(), client->id());
  }
  else if (type == WS_EVT_ERROR)
  {
    Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t *)arg), (char *)data);
  }
  else if (type == WS_EVT_PONG)
  {
    Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len > 0) ? (char *)data : "");
  }
  else if (type == WS_EVT_DATA)
  {
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    String msg = "";
    if (info->final && info->index == 0 && info->len == len)
    {
      // the whole message is in a single frame and we got all of it's data
      // Serial.printf("ws[%s][%u] %s-message[%llu]: ", server->url(), client->id(),
      //     (info->opcode == WS_TEXT) ? "text" : "binary", info->len);

      if (info->opcode == WS_TEXT)
      {
        for (size_t i = 0; i < info->len; i++)
        {
          msg += (char)data[i];
        }
      }
      else
      {
        char buff[3];
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
          Serial.printf("%s\n", msg.c_str());
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
      //message is comprised of multiple frames or the frame is split into multiple packets
      if (info->index == 0)
      {
        if (info->num == 0)
          Serial.printf("ws[%s][%u] %s-message start\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text" : "binary");
        Serial.printf("ws[%s][%u] frame[%u] start[%llu]\n", server->url(), client->id(), info->num, info->len);
      }

      Serial.printf("ws[%s][%u] frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT) ? "text" : "binary", info->index, info->index + len);

      if (info->opcode == WS_TEXT)
      {
        for (size_t i = 0; i < info->len; i++)
        {
          msg += (char)data[i];
        }
      }
      else
      {
        char buff[3];
        for (size_t i = 0; i < info->len; i++)
        {
          sprintf(buff, "%02x ", (uint8_t)data[i]);
          msg += buff;
        }
      }
      Serial.printf("%s\n", msg.c_str());

      if ((info->index + len) == info->len)
      {
        Serial.printf("ws[%s][%u] frame[%u] end[%llu]\n", server->url(), client->id(), info->num, info->len);
        if (info->final)
        {
          Serial.printf("ws[%s][%u] %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT) ? "text" : "binary");
          if (info->message_opcode == WS_TEXT)
          {
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
  bool processed = commandMgr->processCommandLine("WS", &msg);
  if (processed)
  {
    client->text(msg.c_str());
  }
  else
  {
    char r[512];
    snprintf(r, sizeof(r), "Message not processed: %s", msg.c_str());
    r[sizeof(r) - 1] = '\0';
    client->text(r);
  }
}
