#include <CompilationOpts.h>

#ifdef USE_COMM433

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <RadioHead.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "Comm433Service.h"

void Comm433Service::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("comm433");

    ServiceCommands *cmd = commandMgr->getServiceCommands("comm433");
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("rxPin", true)
        .cmd("rxPin")
        .help("--> Set RX pin, set to -1 for no receiving")
        .vMin(-1)
        .vMax(99)
        .ptr(&rxPin)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            rxPin = val;
            *msg = "RX pin set to "; *msg += rxPin; *msg += ", please reboot";
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("txPin", true)
        .cmd("txPin")
        .help("--> Set TX pin, set to -1 for no transmitting")
        .vMin(-1)
        .vMax(99)
        .ptr(&txPin)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            txPin = val;
            *msg = "TX pin set to "; *msg += txPin; *msg += ", please reboot";
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("send", true)
        .cmd("send")
        .help("--> Send data")
        .isPersistent(false)
        .setFn([this](const String &val, bool isLoading, String *msg) -> bool {
            driver->send((uint8_t*)val.c_str(), val.length());
            driver->waitPacketSent(1000);
            *msg = "Sent: "; *msg += val;
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("autosend", true)
        .cmd("autosend")
        .help("--> Send data automatically, each \"autosendPeriod\" millis, set to \"off\" to turn off")
        .isPersistent(false)
        .setFn([this](const String &val, bool isLoading, String *msg) -> bool {
            if (val.equalsIgnoreCase("off")) {
                isAutosend = false;
                this->senderTimer.cancelTimeout();
            } else {
                if (!isAutosend) {
                    isAutosend = true;
                    this->senderTimer.setTimeout(sendPeriod);
                }
                sendData = val;
            }
            *msg = "Sent: "; *msg += val;
            return true;
        })
        .getFn([this](String *val) {
            *val = isAutosend ? sendData : "off";
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("autosendPeriod", true)
        .cmd("autosendPeriod")
        .help("--> Period in millis for autosend")
        .vMin(1)
        .ptr(&sendPeriod)
    );

    rxEventType = eventLoop->getEventType("comm433", "rxData");

    eventLoop->registerTimer(&readerTimer, [this](UEventLoopTimer *loop) -> void {
        uint8_t buf[RH_ASK_MAX_MESSAGE_LEN + 1];
        uint8_t len = sizeof(buf) - 1;
        bool hasMsg = driver->recv(buf, &len);
        if (!hasMsg) {
            Serial.print("-");
            return;
        }

        // we'll add a terminating '\0', so that data can be used as a zero-terminated string
        char *data = new char[sizeof(ReceivedMsg) + len + 1];
        ReceivedMsg *msg = new (data) ReceivedMsg();
        msg->len = len;
        msg->data = data + sizeof(ReceivedMsg);
        memcpy(msg->data, buf, len);
        msg->data[len] = '\0';
        if (logger->isDebug()) {
            logger->debug("Received on 433MHz (on timer): {} bytes: {}", len, LogValue(msg->data, LogValue::StrAction::DO_COPY));
        }

        // set mode to RX again, so that we don't lose any message that can be received right now
        driver->setMode(RHGenericDriver::RHMode::RHModeRx);
digitalWrite(23, HIGH); delayMicroseconds(1); digitalWrite(23, LOW);

Serial.printf("Received on 433MHz: %s\n", msg->data);
        UEvent event;
        event.eventType = rxEventType;
        event.dataPtr = data;
        this->eventLoop->queueEvent(event, [](UEvent *event) {
            ((ReceivedMsg*)(event->dataPtr))->~ReceivedMsg();
            delete (char *)event->dataPtr;
        }, nullptr);
        // if not queued we  may just lose this message
    });

    /**
     * To receive the message as an event:
     * eventLoop->onEvent(rxEventType, [this](UEvent *event) {
     *     ReceivedMsg *msg = (ReceivedMsg *)event->dataPtr;
     *     Serial.printf("Received on 433MHz: %d bytes: %s\n", msg->len, msg->data);
     *     return false;
     * });
     */

    eventLoop->registerTimer(&senderTimer, [this](UEventLoopTimer *loop) -> void {
        if (!isAutosend) {
            return;
        }
        ++sentCnt;
        String data = sendData;
        data.concat(" ");
        data.concat(sentCnt);
        driver->send((uint8_t*)data.c_str(), data.length());
        if (logger->isTrace()) {
            logger->trace("Sent automatically each {} ms: {}\n", sendPeriod, LogValue(data.c_str(), LogValue::StrAction::DO_COPY));
        }
        senderTimer.setTimeout(sendPeriod);
    });

    // defaults

    txPin = COMM433_TX_PIN;
    rxPin = COMM433_RX_PIN;
    isAutosend = false;
    sendPeriod = 1000;
    sendData = "";
    sentCnt = 0;

    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    // init

    pinMode(18, OUTPUT);
    pinMode(21, OUTPUT);
    pinMode(22, OUTPUT);
    pinMode(23, OUTPUT);

    driver = new RH_ASK(1000, rxPin, txPin, -1, false);
    driver->init();
    readerTimer.setInterval(100);

}


#endif
