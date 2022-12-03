#include <CompilationOpts.h>

#ifdef USE_SIM7000
#ifndef INCL_SIM7000_H
#define INCL_SIM7000_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "UartService.h"
#include "LogMgr.h"
#include "Dfa.h"

class Sim7000Service {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, UartService *uart, LogMgr *logMgr);

    struct Sms {
        int slot; // 0 to 9
        bool isValid;
        time_t tm;
        char *text;
        bool isToDelete;
        Sms() { text = nullptr; }
        ~Sms() { delete[] text; }
        void clear() { isValid = false; tm = 0; delete[] text; text = nullptr; isToDelete = false; }
    };

private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    UartService *uart;
    Logger *logger;

    UEventLoopTimer delayedStateTransition;

    int pinPowerKey;
    int pinSleep;
    bool isEnabled;

    bool isInitialized;
    const char *error;
    int initialCount;
    char simPin[5];
    bool isSimUnlocked;
    bool isFirstBackgroundRun;
    int rssi;

    bool rcvExpectingData;
    bool rcvIsMultiLine;
    bool rcvKeepReturnedData;
    const char *rcvPattern1;
    Dfa::Input rcvNextInput1 = Dfa::Input::NONE;
    const char *rcvPattern2;
    Dfa::Input rcvNextInput2 = Dfa::Input::NONE;
    const char *rcvPattern3;
    Dfa::Input rcvNextInput3 = Dfa::Input::NONE;
    const char *rcvPattern4;
    Dfa::Input rcvNextInput4 = Dfa::Input::NONE;
    std::function<Dfa::Input (const char *data)> rcvDynamicPattern;
    bool rcvCaptureEmptyLines;
    bool rcvCaptureMatched;
    std::vector<String> rcvData;

    // state machine for sending a command and receiving the response
    Dfa dfa;
    Dfa::State STARTUP = dfa.nextState("STARTUP");
    Dfa::State INIT_S10 = dfa.nextState("INIT_S10");
    Dfa::State INIT_S20 = dfa.nextState("INIT_S20");
    Dfa::State INIT_S21 = dfa.nextState("INIT_S21");
    Dfa::State INIT_S22 = dfa.nextState("INIT_S22");
    Dfa::State INIT_S40 = dfa.nextState("INIT_S40");
    Dfa::State INIT_S50 = dfa.nextState("INIT_S50");
    Dfa::State INIT_S60 = dfa.nextState("INIT_S60");
    Dfa::State INIT_ERROR = dfa.nextState("INIT_ERROR");
    Dfa::State IDLE = dfa.nextState("IDLE");

    Dfa::State BACKGROUND_S10 = dfa.nextState("BACKGROUND_S10");

    Dfa::Input STARTUP_INITIATE = dfa.nextInput("STARTUP_INITIATE");
    Dfa::Input RECEIVED_OK = dfa.nextInput("RECEIVED_OK");
    Dfa::Input RECEIVED_CLOSED = dfa.nextInput("RECEIVED_CLOSED");
    Dfa::Input RECEIVED_ERROR = dfa.nextInput("RECEIVED_ERROR");
    Dfa::Input RECEIVED_UNEXPECTED = dfa.nextInput("RECEIVED_UNEXPECTED");

    Dfa::State UNLOCK_SIM_S10 = dfa.nextState("UNLOCK_SIM_S10");
    Dfa::Input START_UNLOCK_SIM = dfa.nextInput("START_UNLOCK_SIM");

    String msgToSend;
    String deviceKey;
    String bufferToSend;
    int sentCount; // how many chars have we already sent from the contents of bufferToSend
    String bufferToSendPartial; // what we send at one transmission - length is limited
    int sendMsgRetryCount;
    int msgHologramConfirmationStatus;
    enum MsgStatus {
        NOT_SENT,
        UNCONFIRMED,
        CONFIRMED_OK,
        CONFIRMED_ERROR
    };
    MsgStatus msgStatus; // status of current or last message
    Dfa::State SEND_MSG_S10 = dfa.nextState("SEND_MSG_S10");
    Dfa::State SEND_MSG_S10ERR = dfa.nextState("SEND_MSG_S10ERR");
    Dfa::State SEND_MSG_S20 = dfa.nextState("SEND_MSG_S20");
    Dfa::State SEND_MSG_S30 = dfa.nextState("SEND_MSG_S30");
    Dfa::State SEND_MSG_S40 = dfa.nextState("SEND_MSG_S40");
    Dfa::State SEND_MSG_S50 = dfa.nextState("SEND_MSG_S50");
    Dfa::State SEND_MSG_S60_CONNECT = dfa.nextState("SEND_MSG_S60_CONNECT");
    Dfa::State SEND_MSG_S70 = dfa.nextState("SEND_MSG_S70");
    Dfa::State SEND_MSG_S80 = dfa.nextState("SEND_MSG_S80");
    Dfa::State SEND_MSG_S85 = dfa.nextState("SEND_MSG_S85");
    Dfa::State SEND_MSG_S90 = dfa.nextState("SEND_MSG_S90");
    Dfa::State SEND_MSG_S95 = dfa.nextState("SEND_MSG_S95");
    Dfa::State SEND_MSG_S100 = dfa.nextState("SEND_MSG_S100");
    Dfa::State SEND_MSG_S110 = dfa.nextState("SEND_MSG_S110");
    Dfa::State SEND_MSG_S120 = dfa.nextState("SEND_MSG_S120");
    Dfa::State SEND_MSG_TX_ERR = dfa.nextState("SEND_MSG_TX_ERR");
    Dfa::State SEND_MSG_TERMINATED = dfa.nextState("SEND_MSG_TERMINATED");
    Dfa::State SEND_MSG_RETRY = dfa.nextState("SEND_MSG_RETRY");
    Dfa::Input START_SEND_MSG = dfa.nextInput("START_SEND_MSG");
    Dfa::Input RECEIVED_CONNECT = dfa.nextInput("RECEIVED_CONNECT");

    Sms smss[10]; // Keeps valid SMSs, ordered by Sms::tm. Non-used entries have isValid = false and slot set.
    int smsSortedIndexes[10]; // indexes, sorted by SMS timestamp, all non-valid in the end
    int smsSlot;
    int smsCount; // number of valid SMSs.
    Dfa::State LOAD_SMS_S10 = dfa.nextState("LOAD_SMS_S10");
    Dfa::State LOAD_SMS_S20 = dfa.nextState("LOAD_SMS_S20");
    Dfa::State LOAD_SMS_S30 = dfa.nextState("LOAD_SMS_S30");
    Dfa::State LOAD_SMS_SUCCESS = dfa.nextState("LOAD_SMS_SUCCESS");
    Dfa::State LOAD_SMS_FAILURE = dfa.nextState("LOAD_SMS_FAILURE");
    Dfa::Input START_LOAD_SMS = dfa.nextInput("START_LOAD_SMS");

    Dfa::State DELETE_SMS_S10 = dfa.nextState("DELETE_SMS_S10");
    Dfa::State DELETE_SMS_S20 = dfa.nextState("DELETE_SMS_S20");
    Dfa::State DELETE_SMS_S30 = dfa.nextState("DELETE_SMS_S30");
    Dfa::State DELETE_SMS_TERMINATED = dfa.nextState("DELETE_SMS_TERMINATED");
    Dfa::Input START_DELETE_SMS = dfa.nextInput("START_DELETE_SMS");
    std::function<void(bool success)> smsLoadCallback;
    int smsDeletionsRequested;
    int smsDeletionsPerformed;
    std::function<void(int deletionsRequested, int deletionsPerformed)> smsDeleteTerminatedCallback;

    void initCommands(ServiceCommands *cmd);
    void initDfa();
    void sendAndExpect(const char *cmd, bool isMultiLine, bool isCaptureMatched,
        const char *pattern1, Dfa::Input nextInput1,
        const char *pattern2 = nullptr, Dfa::Input nextInput2 = Dfa::Input::NONE,
        const char *pattern3 = nullptr, Dfa::Input nextInput3 = Dfa::Input::NONE,
        const char *pattern4 = nullptr, Dfa::Input nextInput4 = Dfa::Input::NONE,
        std::function<Dfa::Input (const char *data)> dynamicPattern = nullptr);
    void sendAndExpect(const char *cmd, bool isMultiLine, bool isCaptureMatched,
        std::function<Dfa::Input (const char *data)> dynamicPattern);
    /**
     * Must handle all input on a given state, not only some (not just ENTER_STATE)
     */
    Dfa::TransitionInfo stepExpectingOk(Dfa::Input input, const char *cmd,
        Dfa::State nextState, Dfa::State errorState, const char *errorMsg);
    /**
     * Must handle all input on a given state, not only some (not just ENTER_STATE)
     */
    Dfa::TransitionInfo stepExpectingOk(Dfa::Input input, const char *cmd, int timeoutMillis,
        Dfa::State nextState, Dfa::State errorState, const char *errorMsg);

    bool sendMessageRequested;
    bool sendMessageInProgress;
    const char *sendMessageMsg;
    void *sendMessageArg;
    std::function<void (void *arg, bool sent, bool confirmed)> sendMessageCallback;

public:
    /**
     * The contents of *msg must not change for the duration of the sending.
     */
    void initiateSendMessage(const char *msg, void *arg,
        std::function<void (void *arg, bool sent, bool confirmed)> callback);
    bool isSendingMessage();
    /**
     * Return true if it was possible to initiate loading of SMS (dfa state is IDLE).
     * Caller should retry if returned false.
     **/
    bool initiateLoadSmss(std::function<void(bool isSuccess)> callback);
    int getSmsCount();
    Sms *getSms(int i);
    void markSmsForDeletion(int i);
    int getSmsToDeleteCount();
    void deleteMarkedSms(std::function<void(int deletionsRequested, int deletionsPerformed)> Callback);
    int getRssi();
    const char *getError();
};

#endif
#endif