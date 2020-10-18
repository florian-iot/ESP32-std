#include <CompilationOpts.h>

#ifdef USE_LM75A_TEMP
#ifndef INCL_LM75A_TEMP_H
#define INCL_LM75A_TEMP_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include <M2M_LM75A.h>

class LM75A_TempService {
public:
  void init(UEventLoop *eventLoop, CommandMgr *commandMgr);
private:
  UEventLoop *eventLoop;
  CommandMgr *commandMgr;

  int i2cAddress;

  bool isHardwareOk;
  bool isEnabled;
  bool isLogging;

  UEventLoopTimer  loggingTimer;

  M2M_LM75A *lm75a;

  void enable();
  void disable();
  void startLogging();

};

#endif
#endif
