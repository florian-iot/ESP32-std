#include <CompilationOpts.h>

#ifdef USE_SIM7000

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include "Sim7000.h"
#include "Util.h"
#include <time.h>
#include "pdu.h"

void Sim7000Service::init(UEventLoop *eventLoop, CommandMgr *commandMgr, UartService *uart, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->uart = uart;
    this->logger = logMgr->newLogger("Sim7000");

    ServiceCommands *cmd = commandMgr->getServiceCommands("sim7000");

    initDfa();
    initCommands(cmd);

    isEnabled = false;
    error = nullptr;
    isSimUnlocked = false;
    strcpy(simPin, "1234");

    rcvExpectingData = false;
    rcvIsMultiLine = false;
    rcvKeepReturnedData = false;
    rcvPattern1 = nullptr;
    rcvNextInput1 = Dfa::Input::NONE;
    rcvData.clear();
    rssi = -1;

    deviceKey = "Hn*kRBjr";
    sendMsgRetryCount = 0;
    msgToSend.clear();
    bufferToSend.clear();

    pinPowerKey = -1;
    pinSleep = -1;

    for (int i = 0; i < 10; i++) {
        smss[i].slot = i;
        smss[i].isValid = false;
        smss[i].text = nullptr;
        smss[i].isToDelete = false;
        smsSortedIndexes[i] = i;
    }
    smsCount = 0;
    smsDeleteTerminatedCallback = nullptr;

    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    if (pinSleep != -1) {
        pinMode(pinSleep, INPUT);
    }
    if (pinPowerKey != -1) {
        pinMode(pinPowerKey, INPUT); // PowerKey set to high causes power off (the breakboard has a pulldown)
    }

    eventLoop->onEvent(eventLoop->getEventType("UART", "rxData"), [this](UEvent *event) {
        char *data = (char *)event->dataPtr;
        int len = strlen(data);

        if (!rcvExpectingData) {
            logger->warn("Received data while not expecting data: \"{}\"", data);
            return true;
        }

        if (logger->isDebug()) {
            logger->debug("Received {} bytes: \"{}\"", len, data);
        }

        bool matched = false;
        Dfa::Input input = Dfa::Input::NONE;
        if (rcvPattern1 != nullptr && strcmp(rcvPattern1, data) == 0) {
            rcvExpectingData = false;
            logger->debug("    Matched pattern1");
            matched = true;
            input = rcvNextInput1;
        } else if (rcvPattern2 != nullptr && strcmp(rcvPattern2, data) == 0) {
            rcvExpectingData = false;
            logger->debug("    Matched pattern2");
            matched = true;
            input = rcvNextInput2;
         } else if (rcvPattern3 != nullptr && strcmp(rcvPattern3, data) == 0) {
            rcvExpectingData = false;
            logger->debug("    Matched pattern3");
            matched = true;
            input = rcvNextInput3;
        } else if (rcvPattern4 != nullptr && strcmp(rcvPattern4, data) == 0) {
            rcvExpectingData = false;
            logger->debug("    Matched pattern4");
            matched = true;
            input = rcvNextInput4;
        } else if (rcvDynamicPattern != nullptr) {
            input = rcvDynamicPattern(data);
            if (input != Dfa::Input::NONE) {
                rcvExpectingData = false;
                logger->debug("    Matched dynamic pattern");
                matched = true;
            }
        }
        if (rcvKeepReturnedData) {
            if (rcvData.size() < 100) { // don't capture more than 100 rows
                if (len == 0) {
                    if (rcvCaptureEmptyLines) { // add empty lines only if rcvCaptureEmptyLines
                        rcvData.push_back(String(""));
                    } // else don't capture this empty line
                } else {
                    if (!matched || rcvCaptureMatched) { // don't capture the matched line unless rcvCaptureMatched
                        rcvData.push_back(String(data));
                    }
                }
            }
        }

        if (matched) {
            dfa.handleInput(input);
        } else {
            logger->debug("    Did not match any pattern");
            // TODO may want to send RECEIVED_UNEXPECTED if non-empty line and if requested
        }

        return true;
    });

}

void Sim7000Service::sendAndExpect(const char *cmd,
        bool isMultiLine, bool isCaptureMatched,
        const char *pattern1, Dfa::Input nextInput1,
        const char *pattern2, Dfa::Input nextInput2,
        const char *pattern3, Dfa::Input nextInput3,
        const char *pattern4, Dfa::Input nextInput4,
        std::function<Dfa::Input (const char *data)> dynamicPattern
        )
{
    rcvExpectingData = true;
    rcvIsMultiLine = isMultiLine;
    rcvKeepReturnedData = isMultiLine; // keep the data only if we'll receive multiple lines
    rcvPattern1 = pattern1;
    rcvNextInput1 = nextInput1;
    rcvPattern2 = pattern2;
    rcvNextInput2 = nextInput2;
    rcvPattern3 = pattern3;
    rcvNextInput3 = nextInput3;
    rcvPattern4 = pattern4;
    rcvNextInput4 = nextInput4;
    rcvDynamicPattern = dynamicPattern;
    rcvData.clear();
    rcvCaptureEmptyLines = false; // may want to have as argument
    rcvCaptureMatched = isCaptureMatched;

    if (cmd != nullptr) {
        if (logger->isDebug()) {
            logger->debug("Sending \"{}\"", cmd);
            Serial.printf("Sending \"%s\"\n", cmd);
        }
        uart->send(cmd);
    }
}

void Sim7000Service::sendAndExpect(const char *cmd,
        bool isMultiLine, bool isCaptureMatched, std::function<Dfa::Input (const char *data)> dynamicPattern)
{
    sendAndExpect(cmd, isMultiLine, isCaptureMatched, nullptr, Dfa::Input::NONE,
        nullptr, Dfa::Input::NONE, nullptr, Dfa::Input::NONE, nullptr, Dfa::Input::NONE, dynamicPattern);
}

/**
 * Must handle all input on a given state, not only some (not just ENTER_STATE)
 */
Dfa::TransitionInfo Sim7000Service::stepExpectingOk(Dfa::Input input, const char *cmd,
    Dfa::State nextState, Dfa::State errorState, const char *errorMsg)
{
    return stepExpectingOk(input, cmd, 5000, nextState, errorState, errorMsg);
}

/**
 * Must handle all input on a given state, not only some (not just ENTER_STATE)
 */
Dfa::TransitionInfo Sim7000Service::stepExpectingOk(Dfa::Input input, const char *cmd, int timeoutMillis,
        Dfa::State nextState, Dfa::State errorState, const char *errorMsg)
{
    if (input.is(Dfa::Input::ENTER_STATE)) {
        sendAndExpect(cmd, false, false, "OK", RECEIVED_OK, "ERROR", RECEIVED_ERROR);
        dfa.setStateTimeout(timeoutMillis);
        return dfa.noTransition();
    } else if (input.is(RECEIVED_OK)) {
        return dfa.transitionTo(nextState);
    } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_ERROR, RECEIVED_UNEXPECTED)) {
        error = errorMsg;
        return dfa.transitionTo(errorState);
    } else {
        return dfa.transitionError();
    }
}

// 8888888b.  8888888888     d8888 
// 888  "Y88b 888           d88888 
// 888    888 888          d88P888 
// 888    888 8888888     d88P 888 
// 888    888 888        d88P  888 
// 888    888 888       d88P   888 
// 888  .d88P 888      d8888888888 
// 8888888P"  888     d88P     888 


void Sim7000Service::initDfa()
{
    dfa.init(eventLoop, logger, STARTUP);

    dfa.onInput([this](Dfa *dfa, Dfa::State state, Dfa::Input input) {

        if (state.is(STARTUP)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                isInitialized = false;
                initialCount = 0;
                isFirstBackgroundRun = true;
                error = nullptr;
                return dfa->transitionTo(INIT_S10);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(INIT_S10)) {

            if (input.is(Dfa::Input::ENTER_STATE, Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                if (initialCount < 10) {
                    ++initialCount;
                    this->uart->clearRx();
                    sendAndExpect("AT\r", false, false, "OK", RECEIVED_OK);
                    dfa->setStateTimeout(2000);
                    return dfa->noTransition();
                } else {
                    // timed out 10 times, couldn't establish connection
                    error = "FATAL:INIT Could not establish connection";
                    return dfa->transitionTo(INIT_ERROR);
                }
            } else if (input.is(RECEIVED_OK)) {
                return dfa->transitionTo(INIT_S20);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(INIT_S20)) {

            // turn off echo
            return stepExpectingOk(input, "ATE0\r", INIT_S21, INIT_ERROR, "FATAL:INIT Timeout or received unexpected data at ATE0");

        } else if (state.is(INIT_S21)) {

            // get current clock (from RTC), log it
            if (input.is(Dfa::Input::ENTER_STATE)) {
                sendAndExpect("AT+CCLK?\r", true, false, "OK", RECEIVED_OK);
                dfa->setStateTimeout(1000);
                return dfa->noTransition();
            } else if (input.is(RECEIVED_OK)) {
                const char *d = (rcvData.size() > 0 ? rcvData[0].c_str() : "<unknown>");
                logger->info("Current time from RTC: {}", d);
                return dfa->transitionTo(INIT_S22);
            } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                error = "FATAL:INIT Timeout or received unexpected data at ATI";
                return dfa->transitionTo(INIT_ERROR);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(INIT_S22)) {

            // accept time from network
            return stepExpectingOk(input, "AT+CLTS=1\r", INIT_S40, INIT_ERROR, "FATAL:INIT Timeout or received unexpected data at AT+CLTS=1");

        } else if (state.is(INIT_S40)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                sendAndExpect("ATI\r", true, false, "OK", RECEIVED_OK);
                dfa->setStateTimeout(1000);
                return dfa->noTransition();
            } else if (input.is(RECEIVED_OK)) {
                const char *d = (rcvData.size() > 0 ? rcvData[0].c_str() : "<unknown>");
                logger->info("Device info: {}", d);
                return dfa->transitionTo(INIT_S50);
            } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                error = "FATAL:INIT Timeout or received unexpected data at ATI";
                return dfa->transitionTo(INIT_ERROR);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(INIT_S50)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                sendAndExpect("AT+CSQ\r", true, false, "OK", RECEIVED_OK);
                dfa->setStateTimeout(1000);
                return dfa->noTransition();
            } else if (input.is(RECEIVED_OK)) {
                if (rcvData.size() > 0 && rcvData[0].startsWith("+CSQ: ")) {
                    rssi = atoi(rcvData[0].c_str() + 6);
                } else {
                    rssi = -1;
                }
                logger->info("RSSI = {}", rssi);
                return dfa->transitionTo(INIT_S60);
            } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                error = "BACKGROUND Timeout or received unexpected data at AT+CSQ?";
                return dfa->transitionTo(INIT_ERROR);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(INIT_S60)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                isInitialized = true;
                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(INIT_ERROR)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                // if a message was queued for sending, reply with cannot send
                if (sendMessageRequested) {
                    logger->error("Call to initiateSendMessage failed because initialization failed");
                    sendMessageCallback(sendMessageArg, false, false);
                
                    sendMessageRequested = false;
                    sendMessageInProgress = false;
                    sendMessageMsg = nullptr;
                    sendMessageArg = nullptr;
                    sendMessageCallback = nullptr;
                }

                dfa->setStateTimeout(1000); // we'll wait a moment and retry initialization
                return dfa->noTransition();
            } else if (input.is(STARTUP_INITIATE, Dfa::Input::TIMEOUT)) {
                return dfa->transitionTo(STARTUP);
            } else {
                return dfa->transitionError();
            }

        /************************************************
         *
         * IDLE
         *
         ************************************************/
        } else if (state.is(IDLE)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                // see if a request is pending
                if (sendMessageRequested) {
                    sendMsgRetryCount = 0;
                    sendMessageInProgress = true;
                    msgToSend = sendMessageMsg;
                    return dfa->transitionTo(SEND_MSG_S10);
                } else {
                    dfa->setStateTimeout(isFirstBackgroundRun ? 1000 : 15 * 60 * 1000); // we'll do background work every 15 min
                    return dfa->noTransition();
                }

            } else if (input.is(START_UNLOCK_SIM)) {
                return dfa->transitionTo(UNLOCK_SIM_S10);
            } else if (input.is(STARTUP_INITIATE)) {
                return dfa->transitionTo(STARTUP);
            } else if (input.is(START_SEND_MSG)) {
                sendMsgRetryCount = 0;
                return dfa->transitionTo(SEND_MSG_S10);
            } else if (input.is(START_LOAD_SMS)) {
                return dfa->transitionTo(LOAD_SMS_S10);
            } else if (input.is(START_DELETE_SMS)) {
                return dfa->transitionTo(DELETE_SMS_S10);
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                isFirstBackgroundRun = false;
                return dfa->transitionTo(BACKGROUND_S10);
            } else {
                return dfa->transitionError();
            }

        /************************************************
         *
         * Background process, runs every 15 minutes while in IDLE
         *
         ************************************************/
        } else if (state.is(BACKGROUND_S10)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                sendAndExpect("AT+CSQ\r", true, false, "OK", RECEIVED_OK);
                dfa->setStateTimeout(1000);
                return dfa->noTransition();
            } else if (input.is(RECEIVED_OK)) {
                if (rcvData.size() > 0 && rcvData[0].startsWith("+CSQ: ")) {
                    rssi = atoi(rcvData[0].c_str() + 6);
                } else {
                    rssi = -1;
                }
                logger->info("RSSI = {}", rssi);
                return dfa->transitionTo(IDLE);
            } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                error = "BACKGROUND Timeout or received unexpected data at AT+CSQ?";
                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }

        /************************************************
         *
         * Send message via TCP to Hologram
         *
         ************************************************/
        } else if (state.is(SEND_MSG_S10)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                logger->debug(sendMessageInProgress? "SEND_MSG started with callback" : "SEND_MSG started without callback");
                msgStatus = MsgStatus::NOT_SENT;
                sendAndExpect("AT+CIPSTATUS\r", false, false, "STATE: IP INITIAL", RECEIVED_OK, "STATE: TCP CLOSED", RECEIVED_CLOSED);
                dfa->setStateTimeout(1000);
                return dfa->noTransition();
            } else if (input.is(RECEIVED_OK)) {
                return dfa->transitionTo(SEND_MSG_S20);
            } else if (input.is(RECEIVED_CLOSED)) { // no need to initialize IP, go to connecting via TCP
                return dfa->transitionTo(SEND_MSG_S60_CONNECT);
            } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                return dfa->transitionTo(SEND_MSG_S10ERR);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SEND_MSG_S10ERR)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                ++sendMsgRetryCount;
                if (sendMsgRetryCount < 10) {
                    sendAndExpect("AT+CIPSHUT\r", false, false, "SHUT OK", RECEIVED_OK);
                    dfa->setStateTimeout(65000);
                    return dfa->noTransition();
                } else {
                    error = "SEND_MSG Error initiating IP connection, abandoning after several retries";
                    return dfa->transitionTo(SEND_MSG_TERMINATED);
                }
            } else if (input.is(RECEIVED_OK)) {
                return dfa->transitionTo(SEND_MSG_S10);
            } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                error = "SEND_MSG Error resetting IP connection for retry";
                return dfa->transitionTo(SEND_MSG_TERMINATED);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SEND_MSG_S20)) {

            return stepExpectingOk(input, "AT+CSTT=\"hologram\"\r", SEND_MSG_S30,
                SEND_MSG_TERMINATED, "SEND_MSG Timeout or received unexpected data at AT+CSTT");

        } else if (state.is(SEND_MSG_S30)) {

            return stepExpectingOk(input, "AT+CIICR\r", 85000, SEND_MSG_S40,
                SEND_MSG_TERMINATED, "SEND_MSG Timeout or received unexpected data at AT+CIICR");

        } else if (state.is(SEND_MSG_S40)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                sendAndExpect("AT+CIFSR\r", true, true,
                        nullptr, Dfa::Input::NONE, nullptr, Dfa::Input::NONE, nullptr, Dfa::Input::NONE, nullptr, Dfa::Input::NONE,
                    [this](const char *data) {
                        int len = strlen(data);
                        if (len < 2) {
                            return Dfa::Input::NONE;
                        }
                        for (int i = 0; i < len; i++) {
                            if ((data[i] > '9' || data[i] < '0') && data[i] != '.') {
                                return Dfa::Input::NONE;
                            }
                        }
                        return RECEIVED_OK;
                    });
                dfa->setStateTimeout(5000);
                return dfa->noTransition();
            } else if (input.is(RECEIVED_OK)) {
                const char *ip = (rcvData.size() > 0 ? rcvData[0].c_str() : "<unknown>");
                logger->debug("Received IP address {}", ip);
                return dfa->transitionTo(SEND_MSG_S50);
            } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                error = "SEND_MSG Timeout or received unexpected data at AT+CIFSR";
                return dfa->transitionTo(SEND_MSG_TERMINATED);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SEND_MSG_S50)) {

            return stepExpectingOk(input, "AT+CIPSPRT=0\r",
                SEND_MSG_S60_CONNECT, SEND_MSG_TERMINATED, "SEND_MSG Timeout or received unexpected data at AT+CIPSPRT");

        } else if (state.is(SEND_MSG_S60_CONNECT)) {

            return stepExpectingOk(input, "AT+CIPSTART=\"TCP\",\"cloudsocket.hologram.io\",9999\r",
                SEND_MSG_S70, SEND_MSG_TERMINATED, "SEND_MSG Timeout or received unexpected data at AT+CIPSTART");

        } else if (state.is(SEND_MSG_S70)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                sendAndExpect(nullptr, false, false, "CONNECT OK", RECEIVED_OK, "ALREADY CONNECT", RECEIVED_OK);
                dfa->setStateTimeout(160 * 1000); // 160s
                return dfa->noTransition();
            } else if (input.is(RECEIVED_OK)) {
                return dfa->transitionTo(SEND_MSG_S80);
            } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                error = "SEND_MSG Timeout or received unexpected data waiting for CONNECT OK";
                return dfa->transitionTo(SEND_MSG_TERMINATED);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SEND_MSG_S80)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {

                DynamicJsonBuffer jsonBuf;
                JsonObject& data = jsonBuf.createObject();
                data["k"] = deviceKey;
                data["t"] = "FALOUC_TEST";
                data["d"] = msgToSend;
                bufferToSend.clear();
                data.printTo(bufferToSend);
                sentCount = 0; // prepare for sending bufferToSend

                return dfa->transitionTo(SEND_MSG_S85);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SEND_MSG_S85)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                // how many bytes shall we send right now? Up to 1300 (TODO check specs)
                int toSend = bufferToSend.length() - sentCount;
                if (toSend > 1300) {
                    toSend = 1300;
                }
                bufferToSendPartial = bufferToSend.substring(sentCount, sentCount + toSend);

                String cmd = "AT+CIPSEND=";
                cmd.concat(bufferToSendPartial.length());
                cmd.concat("\r");
                sendAndExpect(cmd.c_str(), true, false, [this](const char *data) {
                    // we're not supposed to receive anything here, except ERROR (e.g., packet too long)
                    return RECEIVED_ERROR;
                });
                dfa->setStateTimeout(50); // 50 ms, go quickly to send the body
                return dfa->noTransition();
            } else if (input.is(RECEIVED_ERROR)) {
                return dfa->transitionTo(SEND_MSG_TX_ERR);
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                return dfa->transitionTo(SEND_MSG_S90);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SEND_MSG_S90)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                sendAndExpect(bufferToSendPartial.c_str(), true, false, "SEND OK", RECEIVED_OK, "SEND FAIL", RECEIVED_UNEXPECTED, "ERROR", RECEIVED_ERROR, "CLOSED", RECEIVED_UNEXPECTED);
                msgStatus = MsgStatus::UNCONFIRMED;
                dfa->setStateTimeout(10000); // 10 sec timeout, just guessing
                return dfa->noTransition();
            } else if (input.is(RECEIVED_OK)) {
                logger->debug("Sent {} bytes: {}", bufferToSendPartial.length(), bufferToSendPartial.c_str());
                sentCount += bufferToSendPartial.length();
                if (bufferToSend.length() > sentCount) { // still more to send
                    return dfa->transitionTo(SEND_MSG_S95); // need one state transition to go at ENTER_STATE of this very state
                } else {
                    return dfa->transitionTo(SEND_MSG_S100);
                }
            } else if (input.is(RECEIVED_UNEXPECTED, RECEIVED_ERROR, Dfa::Input::TIMEOUT)) {
                return dfa->transitionTo(SEND_MSG_TX_ERR);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SEND_MSG_S95)) { // loop to send next block of data

            if (input.is(Dfa::Input::ENTER_STATE)) {
                return dfa->transitionTo(SEND_MSG_S85);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SEND_MSG_S100)) { // wait for Hologram confirmation of receiving the message

            if (input.is(Dfa::Input::ENTER_STATE)) {
                msgHologramConfirmationStatus = -99;
                sendAndExpect(nullptr, true, false, [this](const char *data) {
                    // expecting [<n>,0]
                    // then CLOSED
                    if (strcmp(data, "CLOSED") == 0) {
                        msgHologramConfirmationStatus = -1;
                        return RECEIVED_CLOSED;
                    }
                    if (strcmp(data, "ERROR") == 0) {
                        msgHologramConfirmationStatus = -1;
                        return RECEIVED_ERROR;
                    }
                    int len = strlen(data);
                    if (len < 2 || data[0] != '[' || data[len - 1] != ']') {
                        return Dfa::Input::NONE;
                    }
                    int posComma = 0;
                    for (int i = 1; i < len - 1; i++) {
                        if ((data[i] < '0' || data[i] > '9') && data[i] != ',') {
                            return Dfa::Input::NONE;
                        }
                        if (posComma == 0 && data[i] == ',') {
                            posComma = i;
                        }
                    }
                    if (posComma <= 1 || posComma > 10) {
                        return Dfa::Input::NONE;
                    }
                    char buf[10];
                    strncpy(buf, data + 1, posComma - 1);
                    buf[9] = '\0';
                    msgHologramConfirmationStatus = atoi(buf);
                    return RECEIVED_OK;
                });
                dfa->setStateTimeout(120000); // 2 min timeout for the server to respond (already too much)
                return dfa->noTransition();
            } else if (input.is(RECEIVED_OK)) {
                if (msgHologramConfirmationStatus == 0) {
                    msgStatus = MsgStatus::CONFIRMED_OK;
                    logger->debug("Hologram confirmed message reception for {}", bufferToSend.c_str());
                    return dfa->transitionTo(SEND_MSG_S110); // wait CLOSED
                } else {
                    msgStatus = MsgStatus::CONFIRMED_ERROR;
                    logger->debug("Hologram returned error {} for {}", msgHologramConfirmationStatus, bufferToSend.c_str());
                    return dfa->transitionTo(SEND_MSG_S110); // wait CLOSED
                }
            } else if (input.is(RECEIVED_CLOSED)) {
                logger->debug("Connection closed before receiving confirmation for {}", bufferToSend.c_str());
                return dfa->transitionTo(SEND_MSG_TERMINATED);
            } else if (input.is(RECEIVED_ERROR)) {
                logger->debug("Received ERROR before receiving confirmation for {}", bufferToSend.c_str());
                return dfa->transitionTo(SEND_MSG_TX_ERR);
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                logger->error("Timed out waiting for Hologram confirmation for {}", bufferToSend.c_str());
                return dfa->transitionTo(SEND_MSG_S120); // close
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SEND_MSG_S110)) { // wait for Hologram to close connection

            if (input.is(Dfa::Input::ENTER_STATE)) {
                sendAndExpect(nullptr, true, false, "CLOSED", RECEIVED_CLOSED);
                dfa->setStateTimeout(10000);
                return dfa->noTransition();
            } else if (input.is(RECEIVED_CLOSED)) {
                return dfa->transitionTo(SEND_MSG_TERMINATED);
            } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                logger->warn("Timed out waiting for Hologram to close connection");
                return dfa->transitionTo(SEND_MSG_S120); // close
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SEND_MSG_TX_ERR)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                String data("");
                for (int i = 0; i < rcvData.size(); i++) {
                    if (i > 0) {
                        data.concat("\n");
                        data.concat(rcvData[i]);
                    }
                }
                logger->error("Error sending message {}: {}", bufferToSend.c_str(),
                    data.c_str());
                return dfa->transitionTo(SEND_MSG_S120);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SEND_MSG_S120)) { // close connection

            if (input.is(Dfa::Input::ENTER_STATE)) {
                sendAndExpect("AT+CIPCLOSE\r", false, false, "OK", RECEIVED_OK, "ERROR", RECEIVED_ERROR);
                dfa->setStateTimeout(10000);
                return dfa->noTransition();
            } else if (input.is(RECEIVED_OK)) {
                return dfa->transitionTo(SEND_MSG_TERMINATED);
            } else if (input.is(RECEIVED_ERROR)) {
                logger->warn("Error closing connection, ignored");
                return dfa->transitionTo(SEND_MSG_TERMINATED);
            } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                logger->warn("Timed out closing connection");
                return dfa->transitionTo(SEND_MSG_TERMINATED);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SEND_MSG_TERMINATED)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                if (sendMessageInProgress) {
                    // if we didn't send, we'll retry
                    if (msgStatus == NOT_SENT && sendMsgRetryCount < 10) {
                        return dfa->transitionTo(SEND_MSG_RETRY, 5000); // retry after 5 sec
                    }
                    // else terminate
                    bool sent = false, confirmed = false;
                    switch (msgStatus) {
                        case NOT_SENT: sent = false; confirmed = false; break;
                        case UNCONFIRMED: sent = true; confirmed = false; break;
                        case CONFIRMED_OK: sent = true; confirmed = true; break;
                        case CONFIRMED_ERROR: sent = false; confirmed = false; break;
                    }
                    logger->debug("SEND_MSG terminated, callback call with send = {}, confirmed = {}", sent, confirmed);
                    sendMessageCallback(sendMessageArg, sent, confirmed);
                    sendMessageRequested = false;
                    sendMessageInProgress = false;
                }
                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(SEND_MSG_RETRY)) {

            if (input.is(Dfa::Input::TIMEOUT)) {
                return dfa->transitionTo(SEND_MSG_S10ERR);
            } else {
                return dfa->transitionError();
            }

        /************************************************
         *
         * Load SMS
         *
         ************************************************/
        } else if (state.is(LOAD_SMS_S10)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                smsSlot = 0;
                smsCount = 0;
                for (int i = 0; i < 10; i++) {
                    smss[i].clear();
                    smsSortedIndexes[i] = i;
                }
            }

            // set mode PDU
            // must handle all inputs (cf. function definition), don't put it in an "if"
            return stepExpectingOk(input, "AT+CMGF=0\r",
                LOAD_SMS_S20, LOAD_SMS_FAILURE, "LOAD_SMS Timeout or received unexpected data at AT+CMGF=0");

        } else if (state.is(LOAD_SMS_S20)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                String buf = "AT+CMGR=";
                buf.concat(smsSlot);
                buf.concat("\r");
                sendAndExpect(buf.c_str(), true, false, "OK", RECEIVED_OK);
                dfa->setStateTimeout(5000);
                return dfa->noTransition();
            } else if (input.is(RECEIVED_OK)) {
                const char *data0 = (rcvData.size() > 0 ? rcvData[0].c_str() : "");
                const char *data1 = (rcvData.size() > 1 ? rcvData[1].c_str() : "");
                int data1Len = strlen(data1);
                logger->debug("Read SMS message [{}]\n    {}\n    {}",
                        smsSlot, data0, data1);
                uint8_t pdu[256];
                for (int i = 0; i < data1Len; i += 2) {
                    int nl, nh;
                    if (data1[i] >= '0' && data1[i] <= '9') {
                        nh = data1[i] - '0';
                    } else if (data1[i] >= 'A' && data1[i] <= 'F') {
                        nh = data1[i] - 'A' + 10;
                    } else {
                        nh = 0;
                    }
                    if (data1[i + 1] >= '0' && data1[i + 1] <= '9') {
                        nl = data1[i + 1] - '0';
                    } else if (data1[i + 1] >= 'A' && data1[i + 1] <= 'F') {
                        nl = data1[i + 1] - 'A' + 10;
                    } else {
                        nl = 0;
                    }
                    pdu[i / 2] = (nh << 4) + nl;
                }

                time_t sms_time;
                char phone_number[20];
                char text[256];
                int decodeRc = pdu_decode(pdu, data1Len / 2,
                        &sms_time, phone_number, sizeof(phone_number), text, sizeof(text));

                logger->debug("Decoded SMS message [{}], rc={}:", smsSlot, decodeRc);
                if (decodeRc > 0) {
                    logger->debug("    time: {}\n    phone: {}\n    text: {}",
                            (int)sms_time, phone_number,
                            text);
                }

                Sms *sms = &smss[smsSlot];
                sms->slot = smsSlot;
                if (decodeRc < 0) {
                    sms->isValid = false;
                    sms->text = nullptr;
                    sms->tm = 0;
                } else {
                    sms->isValid = true;
                    sms->text = new char[strlen(text) + 1];
                    strcpy(sms->text, text);
                    sms->tm = sms_time;
                }
                return dfa->transitionTo(LOAD_SMS_S30);

            } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                logger->warn("LOAD_SMS Timeout or received unexpected data at AT+CMGR={}", smsSlot);
                return dfa->transitionTo(LOAD_SMS_FAILURE);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(LOAD_SMS_S30)) { // we need this new state to complete the loop, because we want to re-enter state S20

            if (input.is(Dfa::Input::ENTER_STATE)) {
                ++smsSlot;
                if (smsSlot < 10) {
                    return dfa->transitionTo(LOAD_SMS_S20);
                } else {
                    // we're done loading SMS
                    // sort them by isValid then tm

                    smsCount = 0;
                    for (int i = 0; i < 10; i++) {
                        smsSortedIndexes[i] = i;
                        if (smss[i].isValid) {
                            ++smsCount;
                        }
                    }
                    for (int i = 0; i < 9; i++) {
                        for (int j = i + 1; j < 10; j++) {
                            int idx1 = smsSortedIndexes[i];
                            int idx2 = smsSortedIndexes[j];
                            if ((!smss[idx1].isValid && smss[idx2].isValid)
                                    || (smss[idx1].isValid && smss[idx2].isValid && difftime(smss[idx1].tm, smss[idx2].tm) < 0)) {
                                smsSortedIndexes[i] = idx2;
                                smsSortedIndexes[j] = idx1;
                            }
                        }
                    }
                    return dfa->transitionTo(LOAD_SMS_SUCCESS);
                }
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(LOAD_SMS_FAILURE)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                if (smsLoadCallback) {
                    smsLoadCallback(false);
                    smsLoadCallback = nullptr;
                }
                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(LOAD_SMS_SUCCESS)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                if (smsLoadCallback) {
                    smsLoadCallback(true);
                    smsLoadCallback = nullptr;
                }
                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }

        /************************************************
         *
         * Delete SMS
         *
         ************************************************/
        } else if (state.is(DELETE_SMS_S10)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                smsSlot = 0;
                smsDeletionsRequested = 0;
                smsDeletionsPerformed = 0;
                return dfa->transitionTo(DELETE_SMS_S20);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(DELETE_SMS_S20)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                if (smss[smsSlot].isToDelete) {
                    ++smsDeletionsRequested;
                    String buf = "AT+CMGD=";
                    buf.concat(smsSlot);
                    buf.concat(",0");
                    buf.concat("\r");
                    sendAndExpect(buf.c_str(), true, false, "OK", RECEIVED_OK);
                    dfa->setStateTimeout(6000);
                    return dfa->noTransition();
                } else {
                    return dfa->transitionTo(DELETE_SMS_S30);
                }
            } else if (input.is(RECEIVED_OK)) {
                smss[smsSlot].isValid = false;
                delete[] smss[smsSlot].text;
                smss[smsSlot].text = nullptr;
                smss[smsSlot].tm = 0;
                smss[smsSlot].isToDelete = false;
                ++smsDeletionsPerformed;
                return dfa->transitionTo(DELETE_SMS_S30);
            } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                logger->warn("DELETE_SMS Timeout or received unexpected data at AT+CMGD={}", smsSlot);
                return dfa->transitionTo(DELETE_SMS_TERMINATED);
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(DELETE_SMS_S30)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                ++smsSlot;
                dfa->setStateTimeout(1); // don't call transitionTo() directly, because we would be looping through
                    // the 10 slots immediately, with all calls in stack
                return dfa->noTransition();
            } else if (input.is(Dfa::Input::TIMEOUT)) {
                if (smsSlot < 10) {
                    return dfa->transitionTo(DELETE_SMS_S20);
                } else {
                    return dfa->transitionTo(DELETE_SMS_TERMINATED);
                }
            } else {
                return dfa->transitionError();
            }

        } else if (state.is(DELETE_SMS_TERMINATED)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                if (smsDeleteTerminatedCallback) {
                    smsDeleteTerminatedCallback(smsDeletionsRequested, smsDeletionsPerformed);
                    smsDeleteTerminatedCallback = nullptr;
                }

                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }

        /************************************************
         *
         * Unlock SIM
         *
         ************************************************/
        } else if (state.is(UNLOCK_SIM_S10)) {

            if (input.is(Dfa::Input::ENTER_STATE)) {
                char sendbuff[14] = "AT+CPIN=";
                sendbuff[8] = simPin[0];
                sendbuff[9] = simPin[1];
                sendbuff[10] = simPin[2];
                sendbuff[11] = simPin[3];
                sendbuff[12] = '\r';
                sendbuff[13] = '\0';
                sendAndExpect(sendbuff, false, false, "OK", RECEIVED_OK);
                dfa->setStateTimeout(100);
                return dfa->noTransition();
            } else if (input.is(RECEIVED_OK)) {
                isSimUnlocked = true;
                return dfa->transitionTo(IDLE);
            } else if (input.is(Dfa::Input::TIMEOUT, RECEIVED_UNEXPECTED)) {
                // error on this flow (TODO), go to IDLE
                return dfa->transitionTo(IDLE);
            } else {
                return dfa->transitionError();
            }

        } else {
            return dfa->transitionError();
        }

    });
}


//  .d8888b.   .d88888b.  888b     d888 888b     d888        d8888 888b    888 8888888b.   .d8888b.  
// d88P  Y88b d88P" "Y88b 8888b   d8888 8888b   d8888       d88888 8888b   888 888  "Y88b d88P  Y88b 
// 888    888 888     888 88888b.d88888 88888b.d88888      d88P888 88888b  888 888    888 Y88b.      
// 888        888     888 888Y88888P888 888Y88888P888     d88P 888 888Y88b 888 888    888  "Y888b.   
// 888        888     888 888 Y888P 888 888 Y888P 888    d88P  888 888 Y88b888 888    888     "Y88b. 
// 888    888 888     888 888  Y8P  888 888  Y8P  888   d88P   888 888  Y88888 888    888       "888 
// Y88b  d88P Y88b. .d88P 888   "   888 888   "   888  d8888888888 888   Y8888 888  .d88P Y88b  d88P 
//  "Y8888P"   "Y88888P"  888       888 888       888 d88P     888 888    Y888 8888888P"   "Y8888P"  

void Sim7000Service::initCommands(ServiceCommands *cmd)
{
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("pinSleep", true)
        .cmd("pinSleep")
        .help("--> Set pin connected to Sleep of Sim7000")
        .vMin(-1)
        .vMax(99)
        .ptr(&pinSleep)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            this->pinSleep = val;
            *msg = "Sleep pin set to "; msg->concat(this->pinSleep); msg->concat(", save config and reboot");
            return true;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("pinPowerKey", true)
        .cmd("pinPowerKey")
        .help("--> Set pin connected to PowerKey of Sim7000")
        .vMin(-1)
        .vMax(99)
        .ptr(&pinPowerKey)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            this->pinPowerKey = val;
            *msg = "PowerKey pin set to "; msg->concat(this->pinPowerKey); msg->concat(", save config and reboot");
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("errorMessage", true)
        .cmd("errorMessage")
        .help("--> Last error message, if any error was encountered during last command (or initialization)")
        .isPersistent(false)
        .getFn([this](String *val) {
            *val = (error == nullptr ? "" : error);
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("dfaState", true)
        .cmd("dfaState")
        .help("--> DFA state")
        .isPersistent(false)
        .getFn([this](String *val) {
            *val = "";
            val->concat(dfa.getState().getId());
            val->concat("-");
            val->concat(dfa.stateName(dfa.getState()));
            val->concat(" since "); Util::durationToStr(val, dfa.getMillisInState());
            val->concat(", timeout: ");
            if (dfa.getStateTimeout() <= 0) {
                val->concat("none");
            } else {
                Util::durationToStr(val, dfa.getStateTimeout());
            }
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("dfaInit", true)
        .cmdOn("dfaInit")
        .helpOn("--> Init DFA")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) -> bool {
            if (isInitialized) {
                dfa.queueInputForState(STARTUP_INITIATE, IDLE);
                *msg = "DFA restart requested";
            } else {
                *msg = "Not initialized yet";
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("rssi", true)
        .cmd("rssi")
        .help("--> Signal quality (RSSI). 0: -115 dBm or less, 1: -111 dBm, 2...30: -110... -54 dBm, 31: -52 dBm or greater, 99 not known or not detectable")
        .isPersistent(false)
        .getFn([this]() {
            return rssi;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("smsLoad", true)
        .cmdOn("smsLoad")
        .helpOn("--> Load received SMS messages (then use smsDisplay to show what was loaded)")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) -> bool {
            if (dfa.getState() != IDLE) {
                *msg = "Current state is not IDLE";
                return true;
            }
            smsLoadCallback = nullptr;
            dfa.handleInput(START_LOAD_SMS);
            *msg = "Loading of SMS initiated, use smsDisplay to show what was loaded";
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("smsDisplay", true)
        .cmdOn("smsDisplay")
        .helpOn("--> Display loaded SMS (use smsLoad to load received SMS)")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) -> bool {
            if (dfa.getState() != IDLE) {
                *msg = "Current state is not IDLE";
                return true;
            }
            int cnt = 0;
            for (int i = 0; i < 10; i++) {
                if (smss[smsSortedIndexes[i]].isValid) {
                    ++cnt;
                }
            }
            if (cnt == 0) {
                *msg = "There are no received SMS messages";
                return true;
            }
            *msg = String("Received ") + cnt + " SMS messages:\n";
            for (int i = 0; i < 10; i++) {
                Sms *sms = &smss[smsSortedIndexes[i]];
                if (sms->isValid) {
                    struct tm *ptm = localtime(&sms->tm);
                    char buffer[32];
                    strftime(buffer, 32, "%F %T%z", ptm);
                    *msg += String("    " ) + i + ". (" + smsSortedIndexes[i] + ") at " + buffer + ": " + sms->text + "\n";
                } else {
                    *msg += String("    " ) + i + ". (" + smsSortedIndexes[i] + ") not valid\n";
                }
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("smsDelete", true)
        .cmd("smsDelete")
        .help("--> Delete SMS in a given slot (0-9)")
        .isPersistent(false)
        .vMin(0)
        .vMax(9)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            if (dfa.getState() != IDLE) {
                *msg = "Current state is not IDLE";
                return true;
            }
            markSmsForDeletion(val);
            deleteMarkedSms([this](int deletionsRequested, int deletionsPerformed) {
                if (deletionsRequested != deletionsPerformed) {
                    logger->error("Reqeuested {} SMS deletions, but {} SMS were deleted", deletionsRequested, deletionsPerformed);
                } else {
                    logger->debug("Deleted {} SMSs", deletionsPerformed);
                }
            });
            *msg = "Deleting of SMS initiated";
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("sendMsg", true)
        .cmd("sendMsg")
        .help("--> Send a message to Hologram cloud")
        .isPersistent(false)
        .setFn([this](const String &val, bool isLoading, String *msg) -> bool {
            if (dfa.getState() != IDLE) {
                *msg = "Current state is not IDLE";
                return false;
            }
            this->msgToSend = val;
            dfa.handleInput(START_SEND_MSG);
            *msg = "Sending initiated";
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("sendLongMsg", true)
        .cmd("sendLongMsg")
        .help("sendLongMsg <length> --> Send a test message of the given length to Hologram cloud")
        .vMin(1)
        .vMax(20000)
        .isPersistent(false)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            if (dfa.getState() != IDLE) {
                *msg = "Current state is not IDLE";
                return false;
            }
            this->msgToSend.clear();
            for (int i = 0; i < val; i++) {
                char c;
                if (i == val - 2) {
                    c = '-';
                } else if (i == val - 1) {
                    c = '|';
                } else if ((i % 1000) == 0) {
                    c = 'M';
                } else if ((i % 100) == 0) {
                    c = 'C';
                } else {
                    c = '.';
                }
                this->msgToSend.concat(c);
            }
            dfa.handleInput(START_SEND_MSG);
            *msg = "Sending initiated";
            return true;
        })
    );

}

void Sim7000Service::initiateSendMessage(const char *msg, void *arg, std::function<void (void *arg, bool sent, bool confirmed)> callback)
{
    if (sendMessageRequested) { // can't request twice at the same time
        logger->error("Call to initiateSendMessage failed, another send message is in progress");
        callback(arg, false, false);
    }
    sendMessageRequested = true;
    sendMessageInProgress = false;
    sendMessageMsg = msg;
    sendMessageArg = arg;
    sendMessageCallback = callback;
    if (dfa.getState() == IDLE) {
        sendMessageInProgress = true;
        msgToSend = msg;
        dfa.handleInput(START_SEND_MSG);
    } // else will happen when entering into IDLE state (or if not initialized yet and not being able to initialize)
}

bool Sim7000Service::isSendingMessage()
{
    return sendMessageRequested == true;
}

bool Sim7000Service::initiateLoadSmss(std::function<void(bool isSuccess)> callback)
{
    if (dfa.getState() != IDLE) {
        return false;
    }
    smsLoadCallback = callback;
    dfa.handleInput(START_LOAD_SMS);
    return true;
}

int Sim7000Service::getSmsCount()
{
    return smsCount;
}

Sim7000Service::Sms *Sim7000Service::getSms(int i)
{
    // We'll find all valid smss at the beginning of smsSortedIndexes. We'll return them ordered by smsSortedIndexes.
    return &smss[smsSortedIndexes[i]];
}

void Sim7000Service::markSmsForDeletion(int i)
{
    smss[smsSortedIndexes[i]].isToDelete = true;
}

int Sim7000Service::getSmsToDeleteCount() {
    int d = 0;
    for (int i = 0; i < 10; i++) {
        d += (smss[i].isToDelete ? 1 : 0);
    }
    return d;
}

void Sim7000Service::deleteMarkedSms(std::function<void(int deletionsRequested, int deletionsPerformed)> callback)
{
    int d = getSmsToDeleteCount();
    if (dfa.getState() != IDLE) {
        logger->warn("Delete SMS was called but DFA state is not IDLE");
        callback(d, 0);
        return;
    }
    smsDeleteTerminatedCallback = callback;
    dfa.handleInput(START_DELETE_SMS);
}

int Sim7000Service::getRssi()
{
    return rssi;
}

const char *Sim7000Service::getError()
{
    return error;
}

#endif
