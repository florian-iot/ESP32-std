#include <CompilationOpts.h>

#ifdef USE_TIME
#ifndef INCL_TIME_H
#define INCL_TIME_H

#include <stdlib.h>
#include <time.h>
#include "sys/time.h"
#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"

class TimeService {
public:
  void init(UEventLoop *eventLoop, CommandMgr *commandMgr);
private:
  UEventLoop *eventLoop;
  CommandMgr *commandMgr;

  UEventLoopTimer syncTimer;

  bool isDebug;
  String tzInfo;
  String ntpServer; // set to NONE or empty to disable NTP
  bool isUseRtc;

  bool isSynced;
  bool isNtpActive;
  bool isRtcActive;

  time_t lastSyncedTime;
  unsigned long lastSyncedMillis;

  void sync();

public:
  time_t getNow();

};

#endif
#endif