#include <CompilationOpts.h>

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <freertos/task.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "SystemService.h"
#include "LogMgr.h"
#include "Util.h"
#include "Version.h"

#define TO_STR2(x) #x
#define TO_STR(x) TO_STR2(x)

SystemService::SystemService()
{
    // we start counting uptime from here
    this->rebootTimeTs = millis();
    this->rebootTimeMillis = 0;
}

void SystemService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("i2c");
    isMustReboot = false;
    isShuttingDown = false;

    i2c_sda[0] = -1;
    i2c_scl[0] = -1;
    i2c_freq[0] = 100000;
    i2c_status[0] = NOT_USED;
    i2c_useCount[0] = 0;

    i2c_sda[1] = -1;
    i2c_scl[1] = -1;
    i2c_freq[1] = 100000;
    i2c_status[1] = NOT_USED;
    i2c_useCount[1] = 0;
    uptimeTimer.init(eventLoop, [this](UEventLoopTimer *timer) {
        unsigned long ts = millis();
        rebootTimeMillis += ts - rebootTimeTs;
        rebootTimeTs = ts;
    });
    uptimeTimer.setInterval(24 * 3600 * 1000); // we just want millis() not to overflow

#ifdef USE_SYSTEM_I2C
    ServiceCommands *i2cCmd = commandMgr->getServiceCommands("i2c");
    initI2cCommands(i2cCmd);

    // defaults


    // load config
    String msg;
    bool rc;
    rc = i2cCmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        i2cCmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", i2cCmd->getServiceName(), keyName.c_str());
    }
#endif


    // "system" command
    ServiceCommands *cmdSystem = commandMgr->getServiceCommands("system");
    cmdSystem->registerBoolData(
        ServiceCommands::BoolDataBuilder("reboot", true)
        .cmdOn("reboot")
        .isPersistent(false)
        .includeInStatus(false)
        .helpOn("--> Reboot the microcontroller")
        .setFn([this](bool val, bool isLoading, String *msg) {
            isMustReboot = true;
            return true;
        })
    );
    cmdSystem->registerStringData(
        ServiceCommands::StringDataBuilder("uptime", true)
        .cmd("uptime")
        .help("--> Time since boot")
        .isPersistent(false)
        .getFn([this](String *val) {
            Util::durationToStr(val, uptime());
        })
    );

    cmdSystem->registerStringData(
        ServiceCommands::StringDataBuilder("pins", true)
        .cmd("pins")
        .help("--> Defined pins")
        .isPersistent(false)
        .getFn([this](String *val) {
            val->concat(sysPins.size()); val->concat(" defined pins");
            for (auto i = sysPins.begin(); i != sysPins.end(); i++) {
                val->concat("\n        ");
                val->concat((*i)->getServiceName());
                val->concat(":");
                val->concat((*i)->getName());
                val->concat(" --> Pin ");
                val->concat((*i)->getPin());
            }
        })
    );

    cmdSystem->registerIntData(
        ServiceCommands::IntDataBuilder("stackHighWatermark", true)
        .cmd("stackHighWatermark")
        .help("--> Stack high watermark of the main thread")
        .isPersistent(false)
        .getFn([this]() {
            return uxTaskGetStackHighWaterMark(nullptr);
        })
    );

    cmdSystem->registerStringData(
        ServiceCommands::StringDataBuilder("memory", true)
        .cmd("memory")
        .help("--> Show memory usage")
        .isPersistent(false)
        .includeInStatus(false)
        .getFn([this](String *msg) {
            *msg = StringSumHelper("Heap ") + ESP.getHeapSize()
                + " (" + ESP.getFreeHeap() + " free), Program size " + ESP.getSketchSize() + "\n";
            return true;
        })
    );

    cmdSystem->onAfterStatus([this](String *msg) {
        getInfo(msg);
        *msg += "\n";
        return true;
    });

    // init data structures

    // init hardware

}

void SystemService::getInfo(String *info)
{
  String macAddress = WiFi.macAddress();
  String hostName = (macAddress.length() != 17 ? String("esp32-xx")
      : String("esp32-") + macAddress[12] + macAddress[13] + macAddress[15] + macAddress[16]);

  esp_chip_info_t chipInfo;
  esp_chip_info(&chipInfo);
  const char *flashMode;
  switch (ESP.getFlashChipMode()) {
    case FM_QIO: flashMode = "QIO"; break;
    case FM_QOUT: flashMode = "QOUT"; break;
    case FM_DIO: flashMode = "DIO"; break;
    case FM_DOUT: flashMode = "QOUT"; break;
    case FM_FAST_READ: flashMode = "FAST_READ"; break;
    case FM_SLOW_READ: flashMode = "SLOW_READ"; break;
    case FM_UNKNOWN:
    default: flashMode = "UNKNOWN"; break;
  }
  *info +=
    String(
        "Compiled with configuration: " CONFIG_NAME "\n")
        + "Firmware version " + esp32StdVersion  + ", hostname: " + hostName + "\n"
        + "ESP32 Cores: " + String(chipInfo.cores) + " Frequency: " + String(ESP.getCpuFreqMHz()) + " MHz,\n"
        + "Chip model: "  + ESP.getChipModel() + ", revision: " + ESP.getChipRevision() + ", SDK version: " + ESP.getSdkVersion() + "\n"
        + "Flash mode: " + flashMode + ", Flash speed: " + ESP.getFlashChipSpeed() + "\n"
// it seems heap functions take time, slowing the event loop noticeably
//        + "Heap " + ESP.getHeapSize() + " (" + ESP.getFreeHeap() + " free), Program size " + ESP.getSketchSize() + "\n"
        ;
}

void SystemService::getResetReason(String *info)
{
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
    *info = reasonStr;
}

bool SystemService::isResetReasonUnexpected()
{
    esp_reset_reason_t resetReason = esp_reset_reason();
    return (resetReason == ESP_RST_UNKNOWN
        || resetReason == ESP_RST_PANIC
        || resetReason == ESP_RST_INT_WDT
        || resetReason == ESP_RST_TASK_WDT
        || resetReason == ESP_RST_WDT);
}

void SystemService::setMustReboot()
{
    if (!isMustReboot) {
        isMustReboot = true;
        for (int i = 0; i < shutdownCallbacks.size(); i++) {
            shutdownCallbacks.get(i)();
        }
    }
}

void SystemService::setWillShutDown()
{
    if (!isMustReboot) {
        isShuttingDown = true;
        // don't set isMustReboot to true, as we don't want to reboot -- we're being shut down externally
        for (int i = 0; i < shutdownCallbacks.size(); i++) {
            shutdownCallbacks.get(i)();
        }
    }
}

bool SystemService::mustReboot()
{
    return isMustReboot;
}

int SystemService::onShutdown(std::function<void()>fn)
{
    int h = shutdownCallbacks.add(fn);
    if (isMustReboot || isShuttingDown) {
        fn();
    }
    return h;
}

void SystemService::removeOnShutdown(int handle)
{
    shutdownCallbacks.remove(handle);
}

uint64_t SystemService::uptime()
{
    return rebootTimeMillis + (millis() - rebootTimeTs);
}

#ifdef USE_SYSTEM_I2C

void SystemService::initI2cCommands(ServiceCommands *cmd)
{
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("sda0", true)
        .cmd("sda0")
        .help("i2c port 0 sda pin")
        .ptr(&i2c_sda[0])
        .setFn([this](int val, bool isLoading, String *msg) {
            i2c_sda[0] = val;
            *msg = "Set i2c port 0 sda pin to "; *msg += val; *msg += ", please reboot";
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("sda1", true)
        .cmd("sda1")
        .help("i2c port 1 sda pin")
        .ptr(&i2c_sda[1])
        .setFn([this](int val, bool isLoading, String *msg) {
            i2c_sda[1] = val;
            *msg = "Set i2c port 1 sda pin to "; *msg += val; *msg += ", please reboot";
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("scl0", true)
        .cmd("scl0")
        .help("i2c port 0 scl pin")
        .ptr(&i2c_scl[0])
        .setFn([this](int val, bool isLoading, String *msg) {
            i2c_scl[0] = val;
            *msg = "Set i2c port 0 scl pin to "; *msg += val; *msg += ", please reboot";
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("scl1", true)
        .cmd("scl1")
        .help("i2c port 1 scl pin")
        .ptr(&i2c_scl[1])
        .setFn([this](int val, bool isLoading, String *msg) {
            i2c_scl[1] = val;
            *msg = "Set i2c port 1 scl pin to "; *msg += val; *msg += ", please reboot";
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("freq0", true)
        .cmd("freq0")
        .help("i2c port 0 frequency in Herz")
        .ptr(&i2c_freq[0])
        .setFn([this](int val, bool isLoading, String *msg) {
            i2c_freq[0] = val;
            *msg = "Set i2c port 0 frequency to "; *msg += val; *msg += ", please reboot";
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("freq1", true)
        .cmd("freq1")
        .help("i2c port 1 frequency in Herz")
        .ptr(&i2c_freq[1])
        .setFn([this](int val, bool isLoading, String *msg) {
            i2c_freq[1] = val;
            *msg = "Set i2c port 1 frequency to "; *msg += val; *msg += ", please reboot";
            return true;
        })
    );




    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("scan0", true)
        .cmdOn("scan0")
        .helpOn("--> Scan i2c port 0")
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            scan(0, msg);
            return true;
        })
    );
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("scan1", true)
        .cmdOn("scan1")
        .helpOn("--> Scan i2c port 1")
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            scan(1, msg);
            return true;
        })
    );

}

void SystemService::scan(int port, String *msg)
{
    *msg = "Scanning i2c port "; *msg += port; *msg += "\n";
    int n = 0;
    if (i2c_status[0] == I2cStatus::TWO_WIRE) {
        TwoWire *w = getTwoWire(port);
        if (w == nullptr) {
            *msg += "Error getting TwoWire driver";
            return;
        }
        for (int i = 0; i < 128; i++) {
            w->beginTransmission(i);
            uint8_t r = w->endTransmission();
            if (r == I2C_ERROR_OK) {
                *msg += "    Device at address " + String(i) + " (0x" + String(i, HEX) + ")\n";
                ++n;
            } else if (r != I2C_ERROR_ACK) {
                *msg += "    Error at address " + String(i) + " (0x" + String(i, HEX) + "), error code: " + String(r) + "\n";
            }
        }
    } else {
        bool rc = getIdf(port);
        if (!rc) {
            *msg += "Error getting IDF i2c driver";
            return;
        }
        for (int i = 0; i < 128; i++) {
            i2c_cmd_handle_t cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (i << 1) | 0x0 /* write */, true);
            i2c_master_stop(cmd);
            esp_err_t rc = i2c_master_cmd_begin(port == 0 ? I2C_NUM_0 : I2C_NUM_1, cmd, 1000 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd);
            if (rc == ESP_OK) {
                *msg += "    Device at address " + String(i) + " (0x" + String(i, HEX) + ")\n";
                ++n;
            }
        }
        releaseIdf(port);
    }
    *msg += "Found "; *msg += n; *msg += " devices";
}

TwoWire *SystemService::getTwoWire(int port)
{
    if (i2c_status[port] == I2cStatus::IDF) {
        logger->error("i2c port {} already used as IDF, cannot use it as TwoWire", port);
        return nullptr;
    }
    TwoWire *w = (port == 0 ? &Wire : &Wire1);
    if (i2c_status[port] == I2cStatus::NOT_USED) {
        bool rc = w->begin(i2c_sda[port], i2c_scl[port], i2c_freq[port]);
        if (!rc) {
            logger->error("Error on {}::begin({}, {}, {})", port == 0 ? "Wire" : "Wire1",
                i2c_sda[port], i2c_scl[port], i2c_freq[port]);
            return nullptr;
        }
        i2c_status[port] = I2cStatus::TWO_WIRE;
    }
    ++i2c_useCount[port];
    logger->info("Retrieved i2c TwoWire for port {}, sda: {}, scl:{}, freq: {}, usage count: {}", port,
        i2c_sda[port], i2c_scl[port], i2c_freq[port], i2c_useCount[port]);
    return w;
}

/**
 * There's no way to release a TwoWire, so we just count usage but keep it as I2cStatus::TWO_WIRE
 */
void SystemService::releaseTwoWire(int port)
{
    if (port != 0 && port != 1) {
        logger->error("i2c port must be 0 or 1, but {} was given", port);
        return;
    }
    if (i2c_status[port] != I2cStatus::TWO_WIRE) {
        logger->error("i2c port {} being released is not in use as TwoWire", port);
        return;
    }
    if (i2c_useCount[port] == 0) {
        logger->error("i2c port {} as TwoWire has already been released", port);
        return;
    }
    --i2c_useCount[port];
    logger->info("Released i2c TwoWire for port {}, usage count: {}", port, i2c_useCount[port]);
}

bool SystemService::getIdf(int port)
{
    if (port != 0 && port != 1) {
        logger->error("i2c port must be 0 or 1, but {} was given", port);
        return false;
    }
    return getIdf(port == 0 ? I2C_NUM_0 : I2C_NUM_1);
}

bool SystemService::getIdf(i2c_port_t port)
{
    if (i2c_status[port] == I2cStatus::TWO_WIRE) {
        logger->error("i2c port {} already used as TwoWire, cannot use it as IDF", port);
        return false;
    }

    if (i2c_status[port] == I2cStatus::NOT_USED) {
        // initialize
        logger->debug("Configuring i2c port {} with sda: {}, scl: {}, freq: {}", port, i2c_sda[port], i2c_scl[port], i2c_freq[port]);
        pinMode(i2c_sda[port], OUTPUT);
        pinMode(i2c_scl[port], OUTPUT);
        i2c_config_t conf = { };
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = (gpio_num_t)i2c_sda[port];
        conf.scl_io_num = (gpio_num_t)i2c_scl[port];
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = i2c_freq[port];
        int rc = i2c_param_config(port, &conf);
        if (rc != ESP_OK) {
            logger->error("Error configuring i2c on port {}: {}: {}", port, rc, esp_err_to_name(rc));
            return rc;
        }
        rc = i2c_driver_install(port, conf.mode, 0, 0, 0);
        if (rc != ESP_OK) {
            logger->error("Error installing i2c driver on port {}: {}: {}", port, rc, esp_err_to_name(rc));
            return false;
        }
        i2c_status[port] = I2cStatus::IDF;
    }
    ++i2c_useCount[port];
    logger->info("Retrieved i2c IDF for port {}, sda: {}, scl:{}, freq: {}, usage count: {}", port,
        i2c_sda[port], i2c_scl[port], i2c_freq[port], i2c_useCount[port]);
    return true;
}

void SystemService::releaseIdf(int port)
{
    if (port != 0 && port != 1) {
        logger->error("i2c port must be 0 or 1, but {} was given", port);
        return;
    }
    releaseIdf(port == 0 ? I2C_NUM_0 : I2C_NUM_1);
}

void SystemService::releaseIdf(i2c_port_t port)
{
    if (i2c_status[port] != I2cStatus::IDF) {
        logger->error("i2c port {} being released is not in use as IDF", port);
        return;
    }
    --i2c_useCount[port];
    if (i2c_useCount[port] == 0) {
        i2c_driver_delete(port);
        i2c_status[port] = I2cStatus::NOT_USED;
    }
    logger->info("Released i2c IDF port {}, usage count: {}", port, i2c_useCount[port]);
}

SysPin SystemService::registerSysPin(const char *service, const char *name)
{
    SysPinData *d = new SysPinData(service, name);
    sysPins.push_back(d);
    return SysPin(d);
}


#endif // USE_SYSTEM_I2C
