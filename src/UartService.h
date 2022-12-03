#ifndef INCL_UART_SERVICE_H
#define INCL_UART_SERVICE_H

#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"

// Will send an event ("UART", "rxLine") on each received data line.
// If no full line (with end of line) is received for a given timeout,
// any current data will be sent without end of line.
//
// Will send an event ("UART", "rxError") if errors are encountered on input.
class UartService {
public:
    // returns after writing data to the buffer, from where the driver
    // sends it to the TX FIFO of UART when there's free space in TX FIFO.
    void send(const char *str);
    void clearRx();

public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    Logger *logger;

    int pinRx;
    int pinTx;

    bool isError;
    String errorMsg;

    Monitor mon;
    QueueHandle_t uartQueue;
    TaskHandle_t uartTask;
    Logger *uartTaskLogger;
    uint32_t eventRxData;
    uint32_t eventRxError;

    static void uartEventTaskFn(void *pvParameters);

    bool error(const char *msg);
    String toAsciiHex(const String &str);
    String toAsciiHex(const char *str);
};


#endif
