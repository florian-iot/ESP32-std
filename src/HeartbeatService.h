#include <CompilationOpts.h>

#ifdef USE_HEARTBEAT
#ifndef INCL_HEARTBEAT_H
#define INCL_HEARTBEAT_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"

class HeartbeatService {
public:
  void init(UEventLoop *eventLoop, CommandMgr *commandMgr);
private:
  UEventLoop *eventLoop;
  CommandMgr *commandMgr;

  UEventLoopTimer  heartbeatTimer;
  
  bool isEnabled;
  bool isSerialDot;
  int ledPin;

  long cnt;
  int state;
  int durations[4];
};

#endif
#endif