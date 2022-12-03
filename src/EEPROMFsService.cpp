#include <CompilationOpts.h>

#ifdef USE_EEPROM_LFS

// debug.h defines info() and other defines as used in Logger
// define the following so that debug.h is not included
#define _COAP_DEBUG_H_

#include <HardwareSerial.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "EEPROMFsService.h"
#include "EEPROMLittleFs.h"
#include "SystemService.h"

void EEPROMFsService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr, SystemService *systemService)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("eepromLfs");
    this->systemService = systemService;

    initCmd();

    isEnabled = false;
    i2cPort = 0;
    i2cAddr = 0x50;
    isFsInitialized = false;

    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    EEPROMLittleFs::EEPROMConfig eeCfg;
    EEPROMLittleFs::LFSConfig lfsCfg;
    eeCfg.size = 32768;
    eeCfg.writePageSize = 64;

    if (isEnabled) {
        rc = systemService->getIdf(i2cPort);
        if (!rc) {
            logger->error("Error getting get the i2c driver, the service will not work");
            isFsInitialized = false;
        } else {
            isFsInitialized = eepromLfs.init(i2cPort, eeCfg, lfsCfg, true);
        }
    } else {
        isFsInitialized = false;
    }
    if (!isFsInitialized) {
        logger->error("Error initializing EEPROM LFS, the service will not work");
    } else {
        logger->info("Initialized EEPROM LittleFs at i2c port {}", i2cPort);
    }
}

void EEPROMFsService::initCmd()
{
    cmd = commandMgr->getServiceCommands("eepromLfs");
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("enable", true)
        .cmdOn("enable")
        .cmdOff("disable")
        .helpOn("--> Enable the service (requires reboot if not enabled upon startup)")
        .helpOff("--> Disable the service")
        .ptr(&isEnabled)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("i2cPort", true)
        .cmd("i2cPort")
        .vMin(0)
        .vMax(1)
        .help("--> Set the i2c port, 0 or 1. Configure the port using the i2c service. Requires reboot.")
        .ptr(&i2cPort)
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("i2cAddr", true)
        .cmd("i2cAddr")
        .vMin(0)
        .vMax(127)
        .help("--> Set the i2c address of the eeprom (requires reboot)")
        .ptr(&i2cAddr)
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("format", true)
        .cmdOn("format")
        .helpOn("--> Format the file system")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) -> bool {
            bool rc = eepromLfs.format();
            if (rc) {
                *msg = "Formatted OK";
            } else {
                *msg = "Formatting failed: ";
                msg->concat("[fs] ");
                this->eepromLfs.getLastError(msg);
                msg->concat(" [driver] ");
                this->eepromLfs.getDriverLastError(msg);
            }
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("lastError", true)
        .cmd("lastError")
        .help("--> Last error reported by eeprom driver")
        .isPersistent(false)
        .getFn([this](String *val) {
            val->concat("[fs] ");
            this->eepromLfs.getLastError(val);
            val->concat(" [driver] ");
            this->eepromLfs.getDriverLastError(val);
        })
    );

#ifdef EEPROM_LFS_ENABLE_TESTS

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("testRead", true)
        .cmdOn("testRead")
        .helpOn("--> Run read test")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) -> bool {
            *msg = "Running read test\n";

            File f = eepromLfs.open("/hello.txt", "w+");
            if (!f) {
                *msg += "Error opening file";
                return true;
            }
            bool rc = f.seek(0, fs::SeekMode::SeekEnd);
            if (!rc) {
                *msg += "Error seeking to end of file";
                return true;
            }
            size_t size = f.position();
            if (!f) {
                *msg += "Error getting file size";
                return true;
            }
            *msg += "File size: "; *msg += size; *msg += "\n";
            f.seek(0);
            if (size > 0) {
                String line = f.readString();
                if (!f) {
                    *msg += "Error reading a line";
                    return true;
                }
                *msg += "Read line: \""; *msg += line; *msg += "\n";
                f.seek(-1, fs::SeekMode::SeekCur);
            }
            f.close();

            *msg += "Executed read test";
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("testWrite", true)
        .cmdOn("testWrite")
        .helpOn("--> Run write test")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) -> bool {
            *msg = "Running write test\n";

            File f = eepromLfs.open("/hello.txt", "w+");
            if (!f) {
                *msg += "Error opening file";
                return true;
            }
            bool rc = f.seek(0, fs::SeekMode::SeekEnd);
            if (!rc) {
                *msg += "Error seeking to end of file";
                return true;
            }
            size_t size = f.position();
            if (!f) {
                *msg += "Error getting file size";
                return true;
            }
            *msg += "File size: "; *msg += size; *msg += "\n";
            f.seek(0);
            if (size > 1) {
                f.seek(size - 1);
            }
            f.println("Hello, World!");
            f.flush();
            f.close();

            *msg += "Executed write test";
            return true;
        })
    );
#endif
#if 0
    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("eeWrite", true)
        .cmd("eeWrite")
        .help("--> Write a string to EEPROM")
        .isPersistent(false)
        .setFn([this](const String &val, bool isLoading, String *msg) -> bool {
            if (!eeDriverInited) {
                *msg = "Driver not initialized, error: ";
                msg->concat(eeDriver.lastErrorStr());
                return true;
            }
            *msg = "Running test\n";
            eeDriver.setDeviceAddress(0x50);
            bool rc = true;
            // write half byte by byte, half a full buffer
            int half = val.length() / 2;
            int i;
            for (i = 0; i < half && rc; i++) {
                char c = val.charAt(i);
                rc = eeDriver.writeByte(100 + i, (uint8_t)c);
            }
            if (!rc) {
                msg->concat("Error writing byte "); msg->concat(i);
                msg->concat(": ");
                msg->concat(eeDriver.lastErrorStr());
                msg->concat("\n");
            }
            if (val.length() - half > 0) {
                rc = eeDriver.write(100 + half, (uint8_t*)val.c_str() + half, val.length() - half);
                if (!rc) {
                    msg->concat("Error writing string: ");
                    msg->concat(eeDriver.lastErrorStr());
                    msg->concat("\n");
                }
            }

            eeDriver.sync();

            // read back
            msg ->concat("Reading back\n");
            for (int i = 0; i < val.length() && rc; i++) {
                char c = val.charAt(i);
                uint8_t data;
                rc = eeDriver.readByte(100 + i, &data);
                if (!rc) {
                    msg->concat("Error reading at address: "); msg->concat(100 + i);
                    msg->concat(": "); msg->concat(eeDriver.lastErrorStr()); msg->concat("\n");
                }
                if (data != (uint8_t)c) {
                    msg->concat("    Read byte error at byte ");
                    msg->concat(i);
                    msg->concat(" -- expected "); msg->concat((int)c); msg->concat(" but read back "); msg->concat((int)data);
                    msg->concat("\n");
                }
            }
            msg ->concat("Reading back a whole buffer\n");
            // read back in one call
            uint8_t buf[val.length() + 1];
            memset(buf, 0, val.length());
            buf[val.length()] = 0x5A; // sentinel
            rc = eeDriver.read(100, buf, val.length());
            if (!rc) {
                msg->concat("Error reading buffer: "); msg->concat(eeDriver.lastErrorStr()); msg->concat("\n");
            } else {
                for (int i = 0; i < val.length() && rc; i++) {
                    char c = val.charAt(i);
                    if ((uint8_t)c != buf[i]) {
                        msg->concat("    Read buffer error at byte ");
                        msg->concat(i);
                        msg->concat(" -- expected "); msg->concat((int)c); msg->concat(" but read back "); msg->concat((int)buf[i]);
                        msg->concat("\n");
                    }
                }
            }
            if (buf[val.length()] != 0x5a) {
                msg->concat("    Read buffer error -- buffer overflow\n");
            }

            return true;
        })
    );
#endif

}

#endif
