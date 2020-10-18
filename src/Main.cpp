#include "CompilationOpts.h"


//**********************

#include <WiFi.h>
#include <WebServer.h>
//#define AC_DEBUG 1
#include <AutoConnect.h>
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

#include "LedService.h"
#include "CommandMgr.h"
#include "Stepper.h"
#include "HeartbeatService.h"
#include "LM75A_TempService.h"

#define TO_STR2(x) #x
#define TO_STR(x) TO_STR2(x)

const char * compilationDefines = "\n"
"    Wifi password in standalone mode: " AUTOCONNECT_PSK "\n"

#ifdef USE_LOGGING
"    USE_LOGGING"
#ifdef LOGGING_ENABLE_TESTS
" (with tests)"
#endif
"\n"
#endif

#ifdef USE_LED
"    USE_LED(FASTLED version " TO_STR(FASTLED_VERSION) ", pins compiled in: " TO_STR(LED_PIN_1) ", " TO_STR(LED_PIN_2) ", " TO_STR(LED_PIN_3)
  ", " TO_STR(LED_PIN_4) ", " TO_STR(LED_PIN_5) ", " TO_STR(LED_PIN_6)
  ", " TO_STR(LED_PIN_7) ", " TO_STR(LED_PIN_8) ", " TO_STR(LED_PIN_9)
  ", " TO_STR(LED_PIN_10) ")\n"
#endif
#ifdef USE_HEARTBEAT
"    USE_HEARTBEAT(pin " TO_STR(HEARTBEAT_PIN) ")\n"
#endif
#ifdef USE_SPIFFS_EDITOR
"    USE_SPIFFS_EDITOR(user " TO_STR(SPIFFS_EDITOR_USER) ", password " TO_STR(SPIFFS_EDITOR_PWD) ")\n"
#endif
#ifdef USE_MONITOR_TEST
"    USE_MONITOR_TEST\n"
#endif
#ifdef USE_EVENT_CHECK_HEAP
"    USE_EVENT_CHECK_HEAP\n"
#endif

#ifdef USE_STEPPER
"    USE_STEPPER(pins: ENABLE=" TO_STR(STEPPER_ENABLE_PIN)
    ", M0/1/2=" TO_STR(STEPPER_M0_PIN) "/" TO_STR(STEPPER_M1_PIN) "/" TO_STR(STEPPER_M2_PIN)
    ", RESET=" TO_STR(STEPPER_RESET_PIN)
    ", SLEEP=" TO_STR(STEPPER_SLEEP_PIN)
    ", STEP=" TO_STR(STEPPER_STEP_PIN)
    ", DIR=" TO_STR(STEPPER_DIR_PIN)
    ", FAULT=" TO_STR(STEPPER_FAULT_PIN)
    ")\n"
#endif

#ifdef USE_LM75A_TEMP
"    USE_LM75A_TEMP(SDA=" TO_STR(LM75A_TEMP_SDA)
    ", SCL=" TO_STR(LM75A_TEMP_SCL)
    ", I2CADDRESS=" TO_STR(LM75A_TEMP_I2CADDRESS)
    ")\n"
#endif

;


using namespace fs;


class Services {
public:
  LogMgr *logMgr;
  LogService *logService;
  AsyncWebServer *server;
  WebServer *autoConnectServer;
  AutoConnect *autoConnectPortal;
  OTA *ota;
  UEventLoop *eventLoop;
  CommandMgr *commandMgr;
  WebSocketsServer *wsServer;
  CommandHttpServer *commandHttpServer;
  Services *services;
  #ifdef USE_HEARTBEAT
  HeartbeatService *heartbeatService;
  #endif
  #ifdef USE_LM75A_TEMP
  LM75A_TempService *tempService;
  #endif
  #ifdef USE_LED
  LedService *ledService;
  #endif
  #ifdef USE_STEPPER
  Stepper *stepper;
  #endif
};

Services services;

int sdaPin;
int sclPin;

long stateTs;
volatile boolean mustReboot = false;

#include <driver/i2c.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include "sdkconfig.h"


void getInfo(String *info)
{
  String macAddress = WiFi.macAddress();
  String hostName = (macAddress.length() != 17 ? String("esp32-xx")
      : String("esp32-") + macAddress[12] + macAddress[13] + macAddress[15] + macAddress[16]);

  esp_chip_info_t chipInfo;
  esp_chip_info(&chipInfo);
  info->concat("Firmware version " __DATE__ " " __TIME__ ", hostname "); info->concat(hostName.c_str()); info->concat("\n");
  (*info) += "ESP32 Cores: " + String(chipInfo.cores) + " Frequency: " + String(ESP.getCpuFreqMHz()) + " MHz, Chip revision: "
    + String(chipInfo.revision) + ",\n"
    + "Heap " + ESP.getHeapSize() + " (" + ESP.getFreeHeap() + " free), Cycles " + ESP.getCycleCount()
    + ", Program size " + ESP.getSketchSize() + "\n"
    + "Compiled with: " + compilationDefines;
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(100);

  String info;
  getInfo(&info);
  Serial.println(info);
  info.clear();

  esp_reset_reason_t resetReason = esp_reset_reason();
  const char *reasonStr;
  switch (resetReason) {
    case ESP_RST_UNKNOWN: reasonStr = "Reset reason can not be determined."; break;
    case ESP_RST_POWERON: reasonStr = "Reset due to power-on event."; break;
    case ESP_RST_EXT: reasonStr = "Reset by external pin (not applicable for ESP32)."; break;
    case ESP_RST_SW: reasonStr = "Software reset via esp_restart."; break;
    case ESP_RST_PANIC: reasonStr = "Software reset due to exception/panic."; break;
    case ESP_RST_INT_WDT: reasonStr = "Reset (software or hardware) due to interrupt watchdog."; break;
    case ESP_RST_TASK_WDT: reasonStr = "Reset due to task watchdog."; break;
    case ESP_RST_WDT: reasonStr = "Reset due to other watchdogs."; break;
    case ESP_RST_DEEPSLEEP: reasonStr = "Reset after exiting deep sleep mode."; break;
    case ESP_RST_BROWNOUT: reasonStr = "Brownout reset (software or hardware)."; break;
    case ESP_RST_SDIO: reasonStr = "Reset over SDIO."; break;
    default: reasonStr = "Reset reason unknown."; break;
  }
  Serial.printf("Reset reason: %s\n", reasonStr);
  if (resetReason == ESP_RST_UNKNOWN
    || resetReason == ESP_RST_PANIC
    || resetReason == ESP_RST_INT_WDT
    || resetReason == ESP_RST_TASK_WDT
    || resetReason == ESP_RST_WDT) {
      int d = 5;
      Serial.printf("Delaying initialization %d seconds because of the reset reason \"%s\"\n",
          d, reasonStr);
      delay(d * 1000);
  }
  delay(1000);
  Serial.printf("Main program, current thread %p\n", xTaskGetCurrentTaskHandle());

  // Initialize services

  Serial.println("Starting service initialization");

  services.logMgr = new LogMgr();
  Serial.println(".... Created LogMgr");
  services.logService = new LogService();
  Serial.println(".... Created LogService");
  services.server = new AsyncWebServer(8080);
  Serial.println(".... Created AsyncWebServer");
  services.autoConnectServer = new WebServer(80);
  Serial.println(".... Created WebServer");
  services.autoConnectPortal = new AutoConnect(*services.autoConnectServer);
  Serial.println(".... Created AutoConnect");
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

  #ifdef USE_HEARTBEAT
  services.heartbeatService = new HeartbeatService();
  Serial.println(".... Created HeartbeatService");
  #endif

  #ifdef USE_LM75A_TEMP
  services.tempService = new LM75A_TempService();
  Serial.println(".... Created LM75A_TempService");
  #endif

  #ifdef USE_LED
  services.ledService = LedService();
  Serial.println(".... Created LedService");
  #endif

  #ifdef USE_STEPPER
  services.stepper = new Stepper();
  Serial.println(".... Created Stepper");
  #endif

  Serial.println("Terminated service creation");

  Serial.println("Initializing LogMgr");
  services.logMgr->init();
  Serial.println("Initialized LogMgr");

#ifdef USE_JS
  duk_context *ctx = duk_create_heap_default();
  duk_eval_string(ctx, "1+2");
  Serial.printf("Javascript: 1+2=%d\n", (int) duk_get_int(ctx, -1));
  duk_destroy_heap(ctx);
#endif

  bool spiffsOk = SPIFFS.begin(false); // true: if not formatted, format
  if (!spiffsOk) {
    Serial.println("SPIFFS not formatted, formatting...");
    spiffsOk = SPIFFS.begin(true);
    Serial.printf("SPIFFS formatting %s\n", spiffsOk ? "OK" : " failure");
  }

  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  Serial.println("Starting AutoConnect");

  AutoConnectConfig cfg;
  cfg.psk = AUTOCONNECT_PSK;

  String macAddress = WiFi.macAddress();
  String hostName = (macAddress.length() != 17 ? String("esp32-xx")
      : String("esp32-") + macAddress[12] + macAddress[13] + macAddress[15] + macAddress[16]);
  cfg.apid = hostName;
  cfg.hostName = hostName;
  cfg.autoReconnect = true; // true: connect to old SSIDs, if not possible to currently selected SSID
  cfg.autoRise = true; // false: disable captive portal
  cfg.immediateStart = false; // true: start captive portal without trying to connect to selected SSID
  cfg.portalTimeout = 60000; // millis timeout of captive portal - portal.begin() will return
                              // even if no connection was made to the portal to set SSID
  cfg.retainPortal = true; // true: keep running the portal even after portal timed out (portalTimeout)
  cfg.boundaryOffset = 0; // defaults to 0, offset in EEPROM data to store credentials


// !!!!!!!!!!!!!!!!! TODO WORK IN PROGRESS

  // see if we have a configuration file that has other values for autoconnect configuration
  if (SPIFFS.exists("/autoconnect.json")) {
    File autoConnectParamsFile = SPIFFS.open("/autoconnect.json", "r");
    DynamicJsonBuffer buf;
    JsonObject &params = buf.parseObject(autoConnectParamsFile);
    autoConnectParamsFile.close();
    if (params.success()) {
      Serial.println("Contents of /autoconnect.json:");
      params.prettyPrintTo(Serial);
      Serial.println();
      JsonObject &autoconnect = params["autoconnect"];
      cfg.psk = autoconnect["psk"] | cfg.psk;
      cfg.autoReconnect = autoconnect["autoReconnect"] | cfg.autoReconnect;
      cfg.autoRise = autoconnect["autoRise"] | cfg.autoRise;
      cfg.immediateStart = autoconnect["immediateStart"] | cfg.immediateStart;
      cfg.portalTimeout = autoconnect["portalTimeout"] | cfg.portalTimeout;
      cfg.retainPortal = autoconnect["retainPortal"] | cfg.retainPortal;
      Serial.printf("Merged from /autoconnect.json:\n"
        "    psk: %s\n"
        "    autoReconnect: %s\n"
        "    autoRise: %s\n"
        "    immediateStart: %s\n"
        "    portalTimeout: %ld\n"
        "    retainPortal: %s\n",
        cfg.psk.c_str(),
        cfg.autoReconnect ? "true" : "false", cfg.autoRise ? "true" : "false",
        cfg.immediateStart ? "true" : "false", cfg.portalTimeout, cfg.retainPortal ? "true" : "false"
      );
    } else {
      Serial.println("Error reading /autoconnect.json, file ignored");
    }
  }

  //#define WIFI_FORCE_STANDALONE
  #ifdef WIFI_FORCE_STANDALONE
  cfg.autoReconnect = false;
  cfg.autoRise = true;
  cfg.immediateStart = true;
  cfg.portalTimeout = 1; // millis timeout of captive portal - portal.begin() will return
  // if set to a high value, any http call that comes in between resets the timeout, so we never time out
  #endif
  //#define WIFI_FORCE_CONNECTED
  #ifdef WIFI_FORCE_CONNECTED
  cfg.autoReconnect = true;
  cfg.autoRise = true;
  cfg.immediateStart = false;
  cfg.portalTimeout = 10000;
  #endif

  services.autoConnectPortal->config(cfg);

  services.autoConnectServer->on("/clear", [&cfg]() {
    AutoConnectCredential acCred(cfg.boundaryOffset);
    int acEntries = acCred.entries();
    for (int i = acEntries - 1; i >= 0; i--) {
      station_config_t config;
      acCred.load(i, &config);
      acCred.del(reinterpret_cast<char*>(&config.ssid));
    }
    services.autoConnectServer->send(200, "text/plain", F("Cleared all saved connection information!"));
  });

  bool isConnected = services.autoConnectPortal->begin();
  Serial.printf("AutoConnect returned, isConnected = %d\n", isConnected);
  if (isConnected) {
    Serial.println("WiFi connected: " + WiFi.localIP().toString());
  } else {
    Serial.println("WiFi not connected, access point address: " + WiFi.softAPIP().toString());
  }

  MDNS.begin(hostName.c_str());
  MDNS.addService("_http", "_tcp", 8080);
//  MDNS.addService("_http", "_tcp", 80); // Looks like mDNS doesn't like two _http services in two different ports

  services.ota->init(services.server, services.logMgr, []() { return true; }, [](bool succeeded) { mustReboot = succeeded; } );
  Serial.println("OTA update service initialized");

  services.commandMgr->init(services.eventLoop);

//  services.service->init(services.eventLoop);

  services.wsServer->init(services.server, services.commandMgr);

  services.commandHttpServer->init(services.server, services.commandMgr);

#ifdef USE_SPIFFS_EDITOR
  Serial.printf("SPIFFS editor available on http://%s.local:8080/edit\n", hostName.c_str());
  services.server->addHandler(new SPIFFSEditor(SPIFFS, SPIFFS_EDITOR_USER, SPIFFS_EDITOR_PWD));
#endif

  services.server->on("/heap", HTTP_GET, [](AsyncWebServerRequest *request) {
    Serial.printf("Serving /heap, current thread %p", xTaskGetCurrentTaskHandle());
    request->send(200, "text/plain", String(ESP.getFreeHeap()));
  });

  services.server->serveStatic("/", SPIFFS, "/").setDefaultFile("index.htm");

  extern void setupServerNotFound(AsyncWebServer *server);

  setupServerNotFound(services.server);

  services.server->begin();
  Serial.println("Async web server initialized at port 8080");

  ServiceCommands *cmd = services.commandMgr->getServiceCommands("i2c");
  cmd->registerIntData(
    ServiceCommands::IntDataBuilder("sdaPin", true)
    .cmd("sdaPin")
    .help("--> Set SDA pin for Wire interface. Valid at next reboot, must be saved.")
    .vMin(1)
    .vMax(99)
    .ptr(&sdaPin)
  );
  cmd->registerIntData(
    ServiceCommands::IntDataBuilder("sclPin", true)
    .cmd("sclPin")
    .help("--> Set SCL pin for Wire interface. Valid at next reboot, must be saved.")
    .vMin(1)
    .vMax(99)
    .ptr(&sclPin)
  );
  cmd->registerBoolData(
    ServiceCommands::BoolDataBuilder("scan", true)
    .isPersistent(false)
    .cmdOn("scan")
    .helpOn("--> Scan the i2c bus")
    .setFn([](int val, bool isLoading, String *msg) -> bool {
      *msg = "Scanning i2c bus:\n";
      int n = 0;
      for (int addr = 1; addr <= 127; addr++) {
        Wire.beginTransmission(addr);
        int r = Wire.endTransmission();
        if (r == I2C_ERROR_OK) {
          *msg += "    Device at address " + String(addr) + " (0x" + String(addr, HEX) + ")\n";
          ++n;
        } else if (r != I2C_ERROR_ACK) {
          *msg += "    Error at address " + String(addr) + " (0x" + String(addr, HEX) + "), error code: " + String(r) + "\n";
        }
      }
      *msg += "End of scan, " + String(n) + " devices found";
      return true;
    })
  );

  sdaPin = WIRE0_SDA;
  sclPin = WIRE0_SCL;

  String msg;
  bool rc;
  rc = cmd->load(nullptr, &msg);
  if (rc) {
    String keyName;
    cmd->getCurrentKeyName(&keyName);
    Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
  }

  // "system" command

  ServiceCommands *cmdSystem = services.commandMgr->getServiceCommands("system");
  cmdSystem->registerBoolData(
    ServiceCommands::BoolDataBuilder("reboot", true)
    .cmdOn("reboot")
    .isPersistent(false)
    .includeInStatus(false)
    .helpOn("--> Reboot the microcontroller")
    .setFn([](bool val, bool isLoading, String *msg) {
      mustReboot = true;
      return true;
    })
  );

  cmdSystem->onAfterStatus([](String *msg) {
      getInfo(msg);
      *msg += "\n";
      return true;
  });

  // eventLoop and commandMgr must have been initialized
  Serial.println("Initializing LogService");
  services.logService->init(services.logMgr, services.eventLoop, services.commandMgr);
  Serial.println("Initialized LogService");

#ifdef USE_HEARTBEAT
  services.heartbeatService->init(services.eventLoop, services.commandMgr);
  Serial.println("Heartbeat service initialized");
#endif

#ifdef USE_LM75A_TEMP
  Wire.begin(sdaPin, sclPin, 100000);
  services.tempService->init(&eventLoop, &commandMgr);
  Serial.println("LM75A Temp service initialized");
#endif

#ifdef USE_LED
  services.ledService->init(&eventLoop, &commandMgr);
  Serial.println("Led service initialized");
#endif

#ifdef USE_STEPPER
  services.stepper->init(services.eventLoop, services.commandMgr, services.logMgr);
  Serial.println("Stepper service initialized");
#endif

  stateTs = millis();

#ifdef USE_HW_SD_CARD

  Serial.println("Testing SD card");
// ATTENTION: pins 13, 14, 15 arre used for JTAG
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

}

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

void loop()
{
  static unsigned lastWsCleanup = millis();

  services.autoConnectPortal->handleClient();
  services.eventLoop->runOnce(100); // delay if nothing is to be done -- so we can run autoConnectPortal.handleClient()

  unsigned tm = millis();
  if (tm - lastWsCleanup >= 1000) {
    services.wsServer->cleanupClients();
    lastWsCleanup = tm;
  }

  if (mustReboot) {
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
