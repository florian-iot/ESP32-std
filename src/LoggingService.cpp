#include <CompilationOpts.h>

#ifdef USE_LOGGING

// #include <HardwareSerial.h>
// #include <SPI.h>
// #include <SD.h>
// #include <ArduinoJson.h>
// #include "UEvent.h"
// #include "CommandMgr.h"
// #include "LoggingService.h"

// using namespace fs;

// void LoggingService::init(UEventLoop *eventLoop, CommandMgr *commandMgr)
// {
//   this->eventLoop = eventLoop;
//   this->commandMgr = commandMgr;

//   eventLoop->registerTimer(&writerTimer);
//   writerTimer.setTimeout([this](UEventLoopTimer *timer) {
//     if (!isEnabled) {
//       return;
//     }
//     timer->setTimeout(5000); // every 5 seconds, less if there's more data to write
//   }, 1L);

//   ServiceCommands *cmd = commandMgr->getServiceCommands("logger");

//   String msg;
//   bool rc;
//   rc = cmd->load(nullptr, &msg);
//   if (rc) {
//     String keyName;
//     cmd->getCurrentKeyName(&keyName);
//     Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
//   }


// }

#endif
