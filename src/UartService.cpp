#include <CompilationOpts.h>

#ifdef USE_UART

//#include "driver/gpio.h"
#include <driver/uart.h>
#include <esp_log.h>
#include <esp_intr_alloc.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "UartService.h"

void UartService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("UartService");

    pinRx = UART_RX_PIN;
    pinTx = UART_TX_PIN;
    isError = false;
    errorMsg = "";

    ServiceCommands *cmd = commandMgr->getServiceCommands("uart");
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("pinRx", true)
        .cmd("pinRx")
        .help("--> Set RX pin at ESP32")
        .vMin(1)
        .vMax(99)
        .ptr(&pinRx)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            pinRx = val;
            *msg = "RX pin set to "; msg->concat(pinRx); msg->concat(", save config and reboot");
            return true;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("pinTx", true)
        .cmd("pinTx")
        .help("--> Set TX pin at ESP32")
        .vMin(1)
        .vMax(99)
        .ptr(&pinTx)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            pinTx = val;
            *msg = "TX pin set to "; msg->concat(pinTx); msg->concat(", save config and reboot");
            return true;
        })
    );
    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("error", true)
        .cmd("error")
        .help("--> Current error message, if any")
        .isPersistent(false)
        .getFn([this](String *msg) -> bool {
            *msg = (this->isError ? this->errorMsg : "");
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("sendln", true)
        .cmd("sendln")
        .help("--> Send a serial command, terminated with \\r")
        .isPersistent(false)
        .setFn([this](const String &val, bool isLoading, String *msg) -> bool {
            const char *data = val.c_str();
            size_t toWrite = val.length();
            size_t written = uart_write_bytes(UART_NUM_2, data, toWrite);
            *msg = "Sent \""; msg->concat(val.c_str()); msg->concat("\", written ");
            msg->concat(written); msg->concat(" out of "); msg->concat(toWrite); msg->concat(" bytes\n");
            if (written > 0) {
                uart_write_bytes(UART_NUM_2, "\r", 1);
            }
            return true;
        })
    );

    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    // prepare intr data structures
    // uartIntrData.init(eventLoop);
    // eventLoop->onEvent(uartIntrData.serialEventType, [this](UEvent *evt) {
    //     logger->debug("Received event {}", evt->eventType);
    //     return true;
    // });

    // initialize uart
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE//, // UART_HW_FLOWCTRL_CTS_RTS,
//        .rx_flow_ctrl_thresh = 122,
    };

    bool ok = uart_param_config(UART_NUM_2, &uart_config) == ESP_OK || error("uart_param_config");
    ok = ok && (uart_set_pin(UART_NUM_2, pinTx, pinRx, -1, -1) == ESP_OK || error("uart_set_pin"));
    ok = ok && (uart_driver_install(UART_NUM_2, 1024, 1024, 10, &uartQueue, 0) == ESP_OK || error("uart_driver_install"));

    // ok = ok && (uart_isr_free(UART_NUM_2) == ESP_OK || error("uart_isr_free"));
	// ok = ok && (uart_isr_register(UART_NUM_2, uartIntrHandler, (void *)this, ESP_INTR_FLAG_IRAM, nullptr) == ESP_OK || error("uart_isr_register"));
	// ok = ok && (uart_enable_rx_intr(UART_NUM_2) == ESP_OK || error("uart_enable_rx_intr"));

    eventRxData = eventLoop->getEventType("UART", "rxData");
    eventRxError = eventLoop->getEventType("UART", "rxError");

    // here we may have isError == true and errorMsg with a value
    if (!isError) {
        uartTaskLogger = logMgr->newLogger("UART task");
        xTaskCreate(uartEventTaskFn, "UART Thread", 4096, this, uxTaskPriorityGet(nullptr) + 1, &uartTask);
    }

}

bool UartService::error(const char *msg)
{
    if (!isError) {
        isError = true;
        errorMsg = msg;
    }
    logger->error("UartService error: {}", msg);
    return false;
}

void UartService::send(const char *str)
{
    uart_write_bytes(UART_NUM_2, str, strlen(str));
}

void UartService::clearRx()
{
    uart_event_t event;
    event.type = (uart_event_type_t)(UART_EVENT_MAX + 1); // UART_CUSTOM_CLEAR_RX
    event.size = 0;
    xQueueSendToBack(uartQueue, &event, (portTickType)portMAX_DELAY);
}

void UartService::uartEventTaskFn(void *arg)
{
    UartService *uart = (UartService *)arg;

    uart_event_t event;
    String rxData;
    for (;;) {
        auto rc = xQueueReceive(uart->uartQueue, (void *)&event, (portTickType)portMAX_DELAY);
        if (!rc) {
            continue;
        }

        const char *eventName;
        switch ((int)event.type) {
            case UART_DATA: eventName = "UART_DATA"; break;
            case UART_FIFO_OVF: eventName = "UART_FIFO_OVF"; break;
            case UART_BUFFER_FULL: eventName = "UART_BUFFER_FULL"; break;
            case UART_BREAK: eventName = "UART_BREAK"; break;
            case UART_PARITY_ERR: eventName = "UART_PARITY_ERR"; break;
            case UART_FRAME_ERR: eventName = "UART_FRAME_ERR"; break;
            case UART_PATTERN_DET: eventName = "UART_PATTERN_DET"; break;
            case UART_EVENT_MAX + 1: eventName = "UART_CUSTOM_CLEAR_RX"; break;

            default: eventName = "<unknown name>s"; break;
        }
        uart->uartTaskLogger->debug("Received event {}: {}", event.type, eventName);
Serial.printf("Received event %d: %s\n", event.type, eventName);
        switch ((int)event.type) {
            // Event of UART receving data
            /* We'd better handler data event fast, there would be much more data events than
            other types of events. If we take too much time on data event, the queue might
            be full. */
            case UART_DATA:
                {
                    size_t initialDataLen = rxData.length();
Serial.printf("    Event UART_DATA: event.size = %d\n", event.size);
                    char buf[256];
                    size_t toRead = event.size;
                    size_t read = 1;
                    while (toRead > 0 && read > 0) {
                        read = uart_read_bytes(UART_NUM_2, (uint8_t *)buf, toRead <= sizeof(buf) - 1 ? toRead : sizeof(buf) - 1, portMAX_DELAY);
                        if (read > 0) {
                            buf[read] = '\0';
Serial.printf("    Read %d bytes: %s\n", read, uart->toAsciiHex(buf).c_str());
                            rxData.concat(buf);
                            toRead -= read;
                        }
                    }
                    if (read <= 0) {
Serial.printf("    Error reading uart, event.size = %d, remain to read %d\n", event.size, toRead);
                        // we have an error
                    } else {
Serial.printf("    Current rxData %d bytes: %s\n", rxData.length(), uart->toAsciiHex(rxData).c_str());
                        // split in full lines, send out full lines as events, keep remaining data in rxData (if any)
                        int sentEvents = 0;
                        size_t start = 0;
                        size_t firstIdxNotSent = 0;
                        for (size_t i = initialDataLen; i < rxData.length(); i++) {
                            if (rxData.charAt(i) == '\n') {
                                size_t end = i;
                                if (i > 0 && rxData.charAt(i - 1) == '\r') {
                                    end = i - 1;
                                }
                                char *str = new char[end - start + 1];
                                memcpy(str, rxData.c_str() + start, end - start);
                                str[end - start] = '\0';
Serial.printf("    Sending event %s with data %s\n", uart->eventLoop->getEventName(uart->eventRxData), uart->toAsciiHex(str).c_str());
                                UEvent event(uart->eventRxData, str);
                                bool queued = uart->eventLoop->queueEvent(
                                        event, [](UEvent *event) { delete[] (const char *)(event->dataPtr); }, nullptr, nullptr);
                                // what if we couldn't queue ? Just lost a line.

                                start = i + 1;
                                firstIdxNotSent = i + 1;
                                ++sentEvents;
                            }
                        } // for each new char
                        if (firstIdxNotSent > 0) {
                            rxData.remove(0, firstIdxNotSent);
                        }
Serial.printf("    Sent %d events, remaining rxData %d bytes: %s\n", sentEvents, rxData.length(), uart->toAsciiHex(rxData).c_str());
                    }
                }
                break;
            // Event of HW FIFO overflow detected
            case UART_FIFO_OVF:
                // If fifo overflow happened, you should consider adding flow control for your application.
                // The ISR has already reset the rx FIFO,
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_2);
                xQueueReset(uart->uartQueue);
                break;
            // Event of UART ring buffer full
            case UART_BUFFER_FULL:
                // If buffer full happened, you should consider encreasing your buffer size
                // As an example, we directly flush the rx buffer here in order to read more data.
                uart_flush_input(UART_NUM_2);
                xQueueReset(uart->uartQueue);
                break;
            //Event of UART RX break detected
            case UART_BREAK:
                break;
            //Event of UART parity check error
            case UART_PARITY_ERR:
                break;
            //Event of UART frame error
            case UART_FRAME_ERR:
                break;
            //UART_PATTERN_DET
            case UART_PATTERN_DET:
                // not implemented
                break;
            case UART_EVENT_MAX + 1:
                uart_flush_input(UART_NUM_2);
                rxData.clear();
                break;
            //Others
            default:
                break;
        }

        if (rxData.length() > 0) {


        }
    }

    vTaskDelete(NULL);
}

String UartService::toAsciiHex(const String &str)
{
    return toAsciiHex(str.c_str());
}

String UartService::toAsciiHex(const char *str)
{
    String s;
    for (const char *p = str; *p != '\0'; p++) {
        char c = *p;
        if (c > ' ' && c < 128) {
            s.concat(c);
        } else {
            switch (c) {
                case '\r': s.concat("\\r"); break;
                case '\n': s.concat("\\n"); break;
                case '\0': s.concat("\\0"); break;
                default:
                    s.concat("\\x");
                    if (c < 0x10) {
                        s.concat("0");
                    }
                    s.concat(String((int)c, 16));
                    break;
            }
        }
    }
    return s;
}


#endif
