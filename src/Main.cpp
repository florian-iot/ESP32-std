#include "CompilationOpts.h"

//**********************

#include <WiFi.h>
#include <WebServer.h>
#include <MDNS.h>
#include <Update.h>
#include <FS.h>
#include <SPIFFS.h>
#include <Hash.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <SPIFFSEditor.h>
#include <esp_task.h>
#ifdef USE_JS
#include <duktape.h>
#endif
#include <ArduinoJson.h>
#include <HardwareSerial.h>
#include <soc/efuse_reg.h>
#include <Wire.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>

#include "OTA.h"
#include "CommandWebSockets.h"
#include "CommandHttpServer.h"
#include "UEvent.h"
#include "LogMgr.h"
#include "Dfa.h"
#include "Util.h"

#include "RebootDetectorService.h"
#include "SystemService.h"
#include "WifiServiceAsync.h"
#include "MqttService.h"
#include "LedService.h"
#include "CommandMgr.h"
#include "ButtonService.h"
#include "BeeperService.h"
#include "Stepper.h"
#include "HeartbeatService.h"
#include "LedIndicatorService.h"
#include "LM75A_TempService.h"
#include "AM2320Service.h"
#include "UartService.h"
#include "Sim7000.h"
#include "ServoService.h"
#include "EEPROMFsService.h"
#include "Comm433Service.h"
#include "RtcPcf8563.h"
#include "STWD100Service.h"
#include "Ina3221Service.h"
#include "IvanSupervisor.h"
#include "LedSphereService.h"
#include "Gpsdo.h"
#include "Ble.h"
#include "Esp32I2c.h"
#include "HeaterService.h"

class Services {
public:
  LogMgr *logMgr = nullptr;
  LogService *logService = nullptr;
#ifdef USE_REBOOT_DETECTOR
  RebootDetectorService *rebootDetector = nullptr;
#endif
  WifiAsyncService *wifiService = nullptr;
#ifdef USE_MQTT
  MqttService *mqttService = nullptr;
#endif
  WebSocketsServer *wsServer = nullptr;
  AsyncWebServer *server = nullptr;
  OTA *ota = nullptr;
  UEventLoop *eventLoop = nullptr;
  CommandMgr *commandMgr = nullptr;
  bool isWsServerInitialized = false;
  CommandHttpServer *commandHttpServer = nullptr;
  SystemService *systemService = nullptr;

  #ifdef USE_BUTTON
  ButtonService *buttonService = nullptr;
  #endif
  #ifdef USE_BEEPER
  BeeperService *beeperService = nullptr;
  #endif
  #ifdef USE_HEARTBEAT
  HeartbeatService *heartbeatService = nullptr;
  #endif
  #ifdef USE_LED_INDICATOR
  LedIndicatorService *ledIndicatorService = nullptr;
  #endif
  #ifdef USE_LM75A_TEMP
  LM75A_TempService *tempService = nullptr;
  #endif
  #ifdef USE_AM2320
  AM2320Service *am2320Service = nullptr;
  #endif
  #ifdef USE_LED
  LedService *ledService = nullptr;
  #endif
  #ifdef USE_STEPPER
  Stepper *stepper = nullptr;
  #endif
  #ifdef USE_UART
  UartService *uart = nullptr;
  #endif
  #ifdef USE_SIM7000
  Sim7000Service *sim7000 = nullptr;
  #endif
  #ifdef USE_SERVO
  ServoService *servo = nullptr;
  #endif
  #ifdef USE_EEPROM_LFS
  EEPROMFsService *eepromLfs = nullptr;
  #endif
  #ifdef USE_COMM433
  Comm433Service *comm433 = nullptr;
  #endif
  #ifdef USE_PCF8563
  RtcPcf8563Service *rtcPcf8563 = nullptr;
  #endif
  #ifdef USE_STWD100
  STWD100Service *stwd100 = nullptr;
  #endif
  #ifdef USE_INA3221
  Ina3221Service *ina3221 = nullptr;
  #endif
  #ifdef USE_BLE
  BleService *ble = nullptr;
  #endif
  #ifdef USE_ESP32I2C
  Esp32I2cService *esp32i2c = nullptr;
  #endif
  #ifdef USE_IVAN_SUPERVISOR
  IvanSupervisorService *ivanSupervisor = nullptr;
  #endif
  #ifdef USE_LED_SPHERE
  LedSphereService *ledSphere = nullptr;
  #endif
  #ifdef USE_GPSDO
  GpsdoFreqCounterService *gpsdo = nullptr;
  #endif
  #ifdef USE_HEATER
  HeaterService *heater = nullptr;
  #endif
};

Services services;
Dfa mainDfa("mainDfa", 1);

Dfa::Input MAIN_WIFI_INITIALIZED = mainDfa.nextInput("MAIN_WIFI_INITIALIZED");

Dfa::State MAIN_INIT_START = mainDfa.nextState("MAIN_INIT_START");
Dfa::State MAIN_INIT_LOOP = mainDfa.nextState("MAIN_INIT_LOOP");
Dfa::State MAIN_INIT_DONE = mainDfa.nextState("MAIN_INIT_DONE");
UEventHandle_t wifiInitializedEventHandler;

long stateTs;

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"

void initWebServer();

void setup() {

  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Starting up!\n");

  services.systemService = new SystemService();
  Serial.println(".... Created SystemService");

  // Initialize services

  Serial.println("Starting service initialization");

  services.logMgr = new LogMgr();
  Serial.println(".... Created LogMgr");
  services.logService = new LogService();
#ifdef USE_REBOOT_DETECTOR
  services.rebootDetector = new RebootDetectorService();
  Serial.println(".... Created RebootDetectorService");
#endif
  Serial.println(".... Created LogService");
  services.server = new AsyncWebServer(8080);
  Serial.println(".... Created AsyncWebServer");
  services.wifiService = new WifiAsyncService();
  Serial.println(".... Created WifiService");
  services.ota = new OTA();
  Serial.println(".... Created OTA");
  services.eventLoop = new UEventLoop("Main");
  Serial.println(".... Created UEventLoop");
  services.commandMgr = new CommandMgr();
  Serial.println(".... Created CommandMgr");
  services.wsServer = new WebSocketsServer();
  Serial.println(".... Created WebSocketsServer");
  services.commandHttpServer = new CommandHttpServer();
  Serial.println(".... Created CommandHttpServer");
  #ifdef USE_MQTT
  services.mqttService = new MqttService();
  Serial.println(".... Created MqttServer");
  #endif

  #ifdef USE_BUTTON
  services.buttonService = new ButtonService();
  Serial.println(".... Created ButtonService");
  #endif

  #ifdef USE_BEEPER
  services.beeperService = new BeeperService();
  Serial.println(".... Created BeeperService");
  #endif

  #ifdef USE_HEARTBEAT
  services.heartbeatService = new HeartbeatService();
  Serial.println(".... Created HeartbeatService");
  #endif

  #ifdef USE_LED_INDICATOR
  services.ledIndicatorService = new LedIndicatorService();
  Serial.println(".... Created LedIndicatorService");
  #endif

  #ifdef USE_LM75A_TEMP
  services.tempService = new LM75A_TempService();
  Serial.println(".... Created LM75A_TempService");
  #endif

  #ifdef USE_AM2320
  services.am2320Service = new AM2320Service();
  Serial.println(".... Created AM2320Service");
  #endif

  #ifdef USE_LED
  services.ledService = new LedService();
  Serial.println(".... Created LedService");
  #endif

  #ifdef USE_STEPPER
  services.stepper = new Stepper();
  Serial.println(".... Created Stepper");
  #endif

  #ifdef USE_UART
  services.uart = new UartService();
  Serial.println(".... Created Uart service");
  #endif

  #ifdef USE_SIM7000
  services.sim7000 = new Sim7000Service();
  Serial.println(".... Created Sim7000 service");
  #endif

  #ifdef USE_SERVO
  services.servo = new ServoService();
  Serial.println(".... Created Servo service");
  #endif

  #ifdef USE_EEPROM_LFS
  services.eepromLfs = new EEPROMFsService();
  Serial.println(".... Created EEPROM LFS ervice");
  #endif

  #ifdef USE_COMM433
  services.comm433 = new Comm433Service();
  Serial.println(".... Created Comm433 service");
  #endif

  #ifdef USE_PCF8563
  services.rtcPcf8563 = new RtcPcf8563Service();
  Serial.println(".... Created RtcPcf8563 service");
  #endif

  #ifdef USE_STWD100
  services.stwd100 = new STWD100Service();
  Serial.println(".... Created STDW100 service");
  #endif

  #ifdef USE_INA3221
  services.ina3221 = new Ina3221Service();
  Serial.println(".... Created Ina3221 service");
  #endif

  #ifdef USE_BLE
  services.ble = new BleService();
  Serial.println(".... Created Ble service");
  #endif

  #ifdef USE_ESP32I2C
  services.esp32i2c = new Esp32I2cService();
  Serial.println(".... Created Esp32I2c service");
  #endif

  #ifdef USE_IVAN_SUPERVISOR
  services.ivanSupervisor = new IvanSupervisorService();
  Serial.println(".... Created IvanSupervisor service");
  #endif

  #ifdef USE_LED_SPHERE
  services.ledSphere = new LedSphereService();
  Serial.println(".... Created LedSphere service");
  #endif

  #ifdef USE_GPSDO
  services.gpsdo = new GpsdoFreqCounterService();
  Serial.println(".... Created Gpsdo service");
  #endif

  #ifdef USE_HEATER
  services.heater = new HeaterService();
  Serial.println(".... Created Heater service");
  #endif

  Serial.println("Terminated service creation");

  bool spiffsOk = SPIFFS.begin(false); // true: if not formatted, format
  if (!spiffsOk) {
    Serial.println("SPIFFS not formatted, formatting...");
    spiffsOk = SPIFFS.begin(true);
    Serial.printf("SPIFFS formatting %s\n", spiffsOk ? "OK" : " failure");
  }
  Serial.println("Initialized SPIFFS");

  services.logMgr->init();
  Serial.println("Initialized LogMgr");

  services.commandMgr->init(services.eventLoop);
  Serial.println("Initialized CommandMgr");

  Serial.println("Initializing Log service...");
  services.logService->init(services.logMgr, services.eventLoop, services.commandMgr);
  Serial.println("Log service intialized");

  // time-critic initializations
#ifdef USE_STWD100
  Serial.println("Initializing STWD100...");
  services.stwd100->init(services.eventLoop, services.commandMgr, services.logMgr);
  Serial.println("STWD100 service initialized");
#endif

  struct ServiceInit {
    std::function<void()> init;
    const char *name;
    // The service will be started in all reboot levels less or equal to this one
    RebootDetectorService::Level rebootLevel;
    ServiceInit(const char *name, RebootDetectorService::Level lvl, std::function<void()> init) {
      this->init = init; rebootLevel = lvl; this->name = name;
    }
  };
  std::vector<ServiceInit> *sList = new std::vector<ServiceInit>(); // allocated because we want it to survive this function and be used in DFA

  sList->push_back(ServiceInit("System", RebootDetectorService::MINIMUM, []() {
      services.systemService->init(services.eventLoop, services.commandMgr, services.logMgr); }));
  #ifdef USE_BUTTON
  sList->push_back(ServiceInit("Button", RebootDetectorService::MINIMUM, []() {
    services.buttonService->init(services.eventLoop, services.commandMgr, services.logMgr, services.systemService); }));
  #endif
  #ifdef USE_EEPROM_LFS
  sList->push_back(ServiceInit("EEPROM LFS", RebootDetectorService::MINIMUM, []() {
      services.eepromLfs->init(services.eventLoop, services.commandMgr, services.logMgr, services.systemService); }));
  #endif
  #ifdef USE_REBOOT_DETECTOR
  sList->push_back(ServiceInit("RebootDetector", RebootDetectorService::MINIMUM, []() {
    fs::FS *fs;
    bool isEeprom;
    #ifdef USE_EEPROM_LFS
    fs = services.eepromLfs->getFs();
    if (fs == nullptr) {
      fs = &SPIFFS;
      isEeprom = false;
    } else {
      isEeprom = true;
    }
    #else
    fs = &SPIFFS;
    isEeprom = false;
    #endif
    Serial.printf("Initializing RebootDetector using %s\n", isEeprom ? "EEPROM FS" : "SPIFFS");
    services.rebootDetector->init(services.eventLoop, services.commandMgr, services.logMgr, services.systemService, fs);
  }));
  #endif
  #ifdef USE_HEARTBEAT
  sList->push_back(ServiceInit("Heartbeat", RebootDetectorService::INFRASTRUCTURE, []() {
      services.heartbeatService->init(services.eventLoop, services.commandMgr, services.logMgr); }));
  #endif
  #ifdef USE_BEEPER
  sList->push_back(ServiceInit("Beeper", RebootDetectorService::INFRASTRUCTURE, []() {
    services.beeperService->init(services.eventLoop, services.commandMgr, services.logMgr, services.systemService); }));
  #endif
  #ifdef USE_LED_INDICATOR
  sList->push_back(ServiceInit("LedIndicator", RebootDetectorService::APPLICATION, []() {
      services.ledIndicatorService->init(services.eventLoop, services.commandMgr, services.logMgr); }));
  #endif
  sList->push_back(ServiceInit("Wifi", RebootDetectorService::INFRASTRUCTURE, []() {
    services.wifiService->init(services.server, services.eventLoop, services.commandMgr, services.logMgr); }));
  sList->push_back(ServiceInit("OTA", RebootDetectorService::INFRASTRUCTURE, []() {
      services.ota->init(services.server, services.logMgr, []() { return true; }, [](bool succeeded) {
        if (succeeded) {
          services.systemService->setMustReboot();
        }
      }); }));
  #ifdef USE_MQTT
  sList->push_back(ServiceInit("Mqtt", RebootDetectorService::INFRASTRUCTURE, []() {
    services.mqttService->init(services.eventLoop, services.commandMgr, services.wifiService, services.logMgr);
  }));
  #endif
  #ifdef USE_LM75A_TEMP
  sList->push_back(ServiceInit("LM75A", RebootDetectorService::APPLICATION, []() {
    services.tempService->init(services.eventLoop, services.commandMgr, services.systemService, services.logMgr);
  }));
  #endif
  #ifdef USE_AM2320
  sList->push_back(ServiceInit("AM2320", RebootDetectorService::APPLICATION, []() {
    services.am2320Service->init(services.eventLoop, services.commandMgr, services.systemService, services.logMgr);
  }));
  #endif
  #ifdef USE_LED
  sList->push_back(ServiceInit("Led", RebootDetectorService::APPLICATION, []() {
    services.ledService->init(services.eventLoop, services.commandMgr, services.logMgr); }));
  #endif
  #ifdef USE_STEPPER
  sList->push_back(ServiceInit("Stepper", RebootDetectorService::APPLICATION, []() {
    services.stepper->init(services.eventLoop, services.commandMgr, services.logMgr); }));
  #endif
  #ifdef USE_UART
  sList->push_back(ServiceInit("UART", RebootDetectorService::APPLICATION, []() {
    services.uart->init(services.eventLoop, services.commandMgr, services.logMgr); }));
  #endif
  #ifdef USE_SIM7000
  sList->push_back(ServiceInit("Sim7000", RebootDetectorService::APPLICATION, []() {
    services.sim7000->init(services.eventLoop, services.commandMgr, services.uart, services.logMgr); }));
  #endif
  #ifdef USE_SERVO
  sList->push_back(ServiceInit("Servo", RebootDetectorService::APPLICATION, []() {
    services.servo->init(services.eventLoop, services.commandMgr, services.logMgr); }));
  #endif
  #ifdef USE_COMM433
  sList->push_back(ServiceInit("Comm433", RebootDetectorService::APPLICATION, []() {
    services.comm433->init(services.eventLoop, services.commandMgr, services.logMgr); }));
  #endif
  #ifdef USE_PCF8563
  sList->push_back(ServiceInit("RtcPcf8563", RebootDetectorService::APPLICATION, []() {
    services.rtcPcf8563->init(services.eventLoop, services.commandMgr, services.logMgr, services.systemService); }));
  #endif
  #ifdef USE_INA3221
  sList->push_back(ServiceInit("Ina3221", RebootDetectorService::APPLICATION, []() {
    services.ina3221->init(services.eventLoop, services.commandMgr, services.logMgr, services.systemService); }));
  #endif
  #ifdef USE_BLE
  sList->push_back(ServiceInit("Ble", RebootDetectorService::APPLICATION, []() {
    services.ble->init(services.eventLoop, services.commandMgr, services.logMgr);
  }));
  #endif
  #ifdef USE_ESP32I2C
  sList->push_back(ServiceInit("Esp32I2C", RebootDetectorService::APPLICATION, []() {
    services.esp32i2c->init(services.eventLoop, services.commandMgr, services.systemService, services.logMgr);
  }));
  #endif
  #ifdef USE_IVAN_SUPERVISOR
  sList->push_back(ServiceInit("IvanSupervisor", RebootDetectorService::APPLICATION, []() {
    services.ivanSupervisor->init(services.eventLoop, services.commandMgr,
      services.systemService, services.rebootDetector,
      services.buttonService, services.eepromLfs, services.wifiService,
      services.rtcPcf8563, services.sim7000, services.ina3221, services.tempService,
#ifdef USE_AM2320
      services.am2320Service,
#endif
#ifdef USE_BLE
      services.ble,
#endif
#ifdef USE_ESP32I2C
      services.esp32i2c,
#endif
#ifdef USE_LED_INDICATOR
      services.ledIndicatorService,
#endif
      services.logMgr); }));
  #endif
  #ifdef USE_LED_SPHERE
  sList->push_back(ServiceInit("LedSphere", RebootDetectorService::APPLICATION, []() {
    services.ledSphere->init(services.eventLoop, services.commandMgr, services.logMgr,
      services.systemService, services.buttonService, services.beeperService,
      services.ledService, services.ota); }));
  #endif
  #ifdef USE_GPSDO
  sList->push_back(ServiceInit("GpsdoFreqCounter", RebootDetectorService::APPLICATION, []() {
    services.gpsdo->init(services.eventLoop, services.systemService,
#ifdef USE_MQTT
      services.mqttService,
#endif
      services.commandMgr, services.logMgr);
  }));
  #endif
  #ifdef USE_HEATER
  sList->push_back(ServiceInit("Heater", RebootDetectorService::APPLICATION, []() {
    services.heater->init(services.eventLoop, services.systemService,
#ifdef USE_MQTT
      services.mqttService,
#endif
      services.commandMgr, services.logMgr);
  }));
  #endif
  sList->push_back(ServiceInit("WebServer", RebootDetectorService::INFRASTRUCTURE, []() {
    auto startWebServerCallback = [](bool isStandalone) {
      Serial.println("Initializing web server after wifi started");
      extern void startWebServer();
      startWebServer();
      Serial.println("Web server initialized after wifi started");
    };
    services.wifiService->onStart(startWebServerCallback);
    // if we're already connected, onStart() won't be called, so call it explicitly
    WifiAsyncService::WifiStatus wifiStatus = services.wifiService->getStatus();
    if (wifiStatus == WifiAsyncService::WifiStatus::CONNECTED
        || wifiStatus == WifiAsyncService::WifiStatus::STANDALONE) {
      Serial.println("Wifi is started, calling web server initialization immediately");
      startWebServerCallback(wifiStatus == WifiAsyncService::WifiStatus::STANDALONE);
    }

    // services.wifiService->onStop([this]() {
    //   Serial.println("Stopping web server");
    //   stopWebServer();
    //   Serial.println("Web server stopped");
    // });
  }));

  // rest of initialization, must keep the event loop running

  mainDfa.init(services.eventLoop, services.logMgr->newLogger("mainDfa"), MAIN_INIT_START);

  struct DfaState {
    int initLoopIdx;
    int isLastServiceInitialized;
  } *dfaState = new DfaState();

#ifdef USE_REBOOT_DETECTOR
  Serial.printf("Reboot detector level: %d (%s)\n",
    services.rebootDetector->getRebootLevel(), services.rebootDetector->getRebootLevelStr());
#endif

  mainDfa.onInput([dfaState, sList](Dfa *dfa, Dfa::State state, Dfa::Input input) {
    if (dfaState->initLoopIdx < sList->size()) {
      Serial.printf("On mainDfa, initialization step %d/%d (%s)\n", dfaState->initLoopIdx, sList->size(),
          (*sList)[dfaState->initLoopIdx].name);
    } else {
      Serial.printf("On mainDfa, initialization step %d/%d (out of range!)\n", dfaState->initLoopIdx, sList->size());
    }

    if (state.is(MAIN_INIT_START)) {

      if (input.is(Dfa::Input::ENTER_STATE)) {
        dfaState->initLoopIdx = 0;
        dfaState->isLastServiceInitialized = false;

        String info;
        services.systemService->getInfo(&info);
        Serial.println(info);
        services.systemService->getResetReason(&info);
        Serial.printf("Reset reason: %s\n", info.c_str());
        if (services.systemService->isResetReasonUnexpected()) {
          int d = 2;
          Serial.printf("Delaying further initialization %d seconds because of the reset reason \"%s\"\n",
                d, info.c_str());
          return dfa->transitionTo(MAIN_INIT_LOOP, d * 1000);
        } else {
          return dfa->transitionTo(MAIN_INIT_LOOP, 1);
        }
      } else {
        return dfa->transitionError();
      }

    } else if (state.is(MAIN_INIT_LOOP)) {

      if (input.is(Dfa::Input::TIMEOUT)) {
        unsigned long tm = millis();

        uint32_t totalHeap = ESP.getHeapSize();
        uint32_t freeHeapBefore = ESP.getFreeHeap();

        ServiceInit *sInit = &(*sList)[dfaState->initLoopIdx];
        #ifdef USE_REBOOT_DETECTOR
        if (services.rebootDetector != nullptr && services.rebootDetector->getRebootLevel() >= sInit->rebootLevel) {
          Serial.printf("Initializing %s...\n", sInit->name);
          sInit->init();
          uint32_t freeHeapAfter = ESP.getFreeHeap();
          Serial.printf("Initialized %s in %d millis, used %d heap memory out of total %d, free heap %d\n",
            sInit->name, (int)(millis() - tm), freeHeapAfter - freeHeapBefore, totalHeap, freeHeapAfter);
        } else {
          Serial.printf("Skipped initialization of %s because of too many quick reboots (reboot level %d %s)\n",
            sInit->name, sInit->rebootLevel >= services.rebootDetector->getRebootLevel(),
            services.rebootDetector->getRebootLevelStr());
        }
        #else
        Serial.printf("Initializing %s...\n", sInit->name);
        sInit->init();
        uint32_t freeHeapAfter = ESP.getFreeHeap();
        Serial.printf("Initialized %s in %d millis, used %d heap memory out of total %d, free heap %d\n",
          sInit->name, (int)(millis() - tm), freeHeapAfter - freeHeapBefore, totalHeap, freeHeapAfter);
        #endif

        ++dfaState->initLoopIdx;

        if (dfaState->initLoopIdx == sList->size()) {
          dfaState->isLastServiceInitialized = true;
          return dfa->transitionTo(MAIN_INIT_DONE);
        } else {
          dfa->setStateTimeout(1);
          return dfa->noTransition();
        }
      } else {
        return dfa->transitionError();
      }

    } else if (state.is(MAIN_INIT_DONE)) {

      if (input.is(Dfa::Input::ENTER_STATE)) {
        Serial.println("Initialization terminated");
// FX        delete dfaState;
        return dfa->noTransition();
      } else {
        return dfa->transitionError();
      }

    } else {

      return dfa->transitionError();

    }
  });

  stateTs = millis();

  Serial.println("End of setup()");

} // end of setup()

#ifdef USE_JS
  duk_context *ctx = duk_create_heap_default();
  duk_eval_string(ctx, "1+2");
  Serial.printf("Javascript: 1+2=%d\n", (int) duk_get_int(ctx, -1));
  duk_destroy_heap(ctx);
#endif


void startWebServer()
{
  if (services.isWsServerInitialized) {
    return;
  }
  Serial.println("Starting web server");

  // Initialize web server
  services.wsServer->init(services.server, services.commandMgr, services.systemService, services.rebootDetector, services.logMgr);
  services.isWsServerInitialized = true;

  services.commandHttpServer->init(services.server, services.commandMgr, services.logMgr);

  String hostName = services.wifiService->getHostName();

#ifdef USE_SPIFFS_EDITOR
  Serial.printf("SPIFFS editor available on http://%s.local:8080/edit\n", hostName.c_str());
  services.server->addHandler(new SPIFFSEditor(SPIFFS, SPIFFS_EDITOR_USER, SPIFFS_EDITOR_PWD));
#endif

  services.server->on("/heap", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.printf("Serving /heap, current thread %p", xTaskGetCurrentTaskHandle());
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

  services.server->serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");
#ifdef USE_EEPROM_LFS
  if (services.eepromLfs->getFs() != nullptr) {
#ifdef USE_SPIFFS_EDITOR
    Serial.printf("EEPROMLittleFs editor available on http://%s.local:8080/edit_eeprom\n", hostName.c_str());
    services.server->addHandler(new SPIFFSEditor(*services.eepromLfs->getFs(), "/edit_eeprom", SPIFFS_EDITOR_USER, SPIFFS_EDITOR_PWD));
#endif
    services.server->serveStatic("/eeprom", *services.eepromLfs->getFs(), "/").setDefaultFile("index.htm");
  } else {
    Serial.printf("EEPROMLittleFs editor not available, as the filesystem is not initialized\n");
  }
#endif

  extern void setupServerNotFound(AsyncWebServer *server);

  setupServerNotFound(services.server);

  services.server->begin();
  Serial.println("Web server started at port 8080");

} // initWebServer()

#ifdef USE_HW_SD_CARD

  Serial.println("Testing SD card");
// ATTENTION: pins 13, 14, 15 are used for JTAG
  int8_t sck = 14;
  int8_t miso = 33;
  int8_t mosi = 13;
  int8_t ss = 15;

  SPIClass spi(HSPI);
  spi.begin(sck, miso, mosi, ss);

  SD.begin(ss, spi, 4000000, "/sd", 5);

  File root = SD.open("/");
  if (root) {
    extern void printDirectory(File dir, int numTabs);
    printDirectory(root, 0);
    root.close();
  } else {
    Serial.println("error opening test.txt");
  }

  Serial.println("Testing SD card finished");
#endif

#ifdef USE_HW_SD_CARD
void printDirectory(File dir, int numTabs) {

  while (true) {
     File entry =  dir.openNextFile();
     if (!entry) {
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');   // we'll have a nice indentation
     }
     // Print the name
     Serial.print(entry.name());
     /* Recurse for directories, otherwise print the file size */
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       /* files have sizes, directories do not */
       Serial.print("\t\t");
       Serial.println(entry.size());
     }
     entry.close();
   }
}
#endif

Logger *mainLogger = nullptr;
unsigned lastWsCleanup = millis();
String serialCommand("");

void loop()
{
  services.eventLoop->runOnce(50);
  if (mainDfa.getState() != MAIN_INIT_DONE) {
    return; // during initialization just run the event loop, nothing else
  }

  unsigned tm = millis();
  if (tm - lastWsCleanup >= 1000) {
    if (services.isWsServerInitialized) {
      services.wsServer->cleanupClients();
    }
    lastWsCleanup = tm;
  }
  if (mainLogger == nullptr) {
    mainLogger = services.logMgr->newLogger("mainDfa");
  }

  while (Serial && Serial.available()) {
    int c = Serial.read();
    if (c == -1) {
      break; // nothing more to read
    }

    // echo
    Serial.write(c);
    Serial.flush();

    if (c == '\n') {
      String cmd = serialCommand;
      cmd.trim();
      if (cmd == "") {
        continue;
      }
      bool processed = services.commandMgr->processCommandLine("Serial", &cmd);
      if (processed) {
        Serial.println(cmd);
      } else {
        Serial.println("Command not processed");
      }
      serialCommand.clear();
    } else if (c == '\b') {
      if (serialCommand.length() > 0) {
        serialCommand.remove(serialCommand.length() - 1);
        Serial.print("^h ^h");
        Serial.flush();
      }
    } else if (c != '\r') { // ignoring \r
      if (serialCommand.length() < 2000) {
        // else command line too long, forget it
        serialCommand.concat((char)c);
      }
    }

  }

  if (services.systemService->mustReboot()) {
    ESP.restart();
  }
}


void setupServerNotFound(AsyncWebServer *server)
{
  server->onNotFound([](AsyncWebServerRequest *request){
    Serial.printf("NOT_FOUND: ");
    if(request->method() == HTTP_GET)
      Serial.printf("GET");
    else if(request->method() == HTTP_POST)
      Serial.printf("POST");
    else if(request->method() == HTTP_DELETE)
      Serial.printf("DELETE");
    else if(request->method() == HTTP_PUT)
      Serial.printf("PUT");
    else if(request->method() == HTTP_PATCH)
      Serial.printf("PATCH");
    else if(request->method() == HTTP_HEAD)
      Serial.printf("HEAD");
    else if(request->method() == HTTP_OPTIONS)
      Serial.printf("OPTIONS");
    else
      Serial.printf("UNKNOWN");
    Serial.printf(" http://%s%s\n", request->host().c_str(), request->url().c_str());

    if(request->contentLength()){
      Serial.printf("_CONTENT_TYPE: %s\n", request->contentType().c_str());
      Serial.printf("_CONTENT_LENGTH: %u\n", request->contentLength());
    }

    int headers = request->headers();
    int i;
    for(i=0;i<headers;i++){
      AsyncWebHeader* h = request->getHeader(i);
      Serial.printf("_HEADER[%s]: %s\n", h->name().c_str(), h->value().c_str());
    }

    int params = request->params();
    for(i=0;i<params;i++){
      AsyncWebParameter* p = request->getParam(i);
      if(p->isFile()){
        Serial.printf("_FILE[%s]: %s, size: %u\n", p->name().c_str(), p->value().c_str(), p->size());
      } else if(p->isPost()){
        Serial.printf("_POST[%s]: %s\n", p->name().c_str(), p->value().c_str());
      } else {
        Serial.printf("_GET[%s]: %s\n", p->name().c_str(), p->value().c_str());
      }
    }

    request->send(404);
  });
  server->onFileUpload([](AsyncWebServerRequest *request, const String& filename, size_t index, uint8_t *data, size_t len, bool final){
    if(!index)
      Serial.printf("UploadStart: %s\n", filename.c_str());
    Serial.printf("%s", (const char*)data);
    if(final)
      Serial.printf("UploadEnd: %s (%u)\n", filename.c_str(), index+len);
  });
  server->onRequestBody([](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
    if(!index)
      Serial.printf("BodyStart: %u\n", total);
    Serial.printf("%s", (const char*)data);
    if(index + len == total)
      Serial.printf("BodyEnd: %u\n", total);
  });
}
