
#include "CommandMgr.h"

#include <ArduinoJson.h>
#include <FS.h>
#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <WString.h>

#include <vector>

CommandMgr::CommandMgr() {
}

CommandMgr::~CommandMgr() {
}

bool CommandMgr::getIntValue(UEvent *event, int *val) {
    long longVal;
    bool rc = getLongValue(event, &longVal);
    if (rc) {
        if ((long)(int)longVal != longVal) {
            String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
            char buf[200];
            snprintf(buf, sizeof(buf), "Bad integer \"%s\"\n", msg->c_str());
            buf[sizeof(buf) - 1] = '\0';
            Serial.println(buf);
            *msg = buf;
            return false;
        } else {
            *val = (int)longVal;
        }
    }
    return rc;
}

bool CommandMgr::getLongValue(UEvent *event, long *val) {
    String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));

    char *endptr;
    int valTmp = strtol(msg->c_str(), &endptr, 0);
    if (*endptr != '\0') {  // bad integer...
        char buf[200];
        snprintf(buf, sizeof(buf), "Bad integer \"%s\"\n", msg->c_str());
        buf[sizeof(buf) - 1] = '\0';
        Serial.println(buf);
        *msg = buf;
        return false;
    }
    *val = valTmp;
    return true;
}

bool CommandMgr::getFloatValue(UEvent *event, float *val) {
    String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));

    char *endptr;
    float valTmp = strtof(msg->c_str(), &endptr);
    if (*endptr != '\0') {  // bad integer...
        char buf[200];
        snprintf(buf, sizeof(buf), "Bad floating point value \"%s\"\n", msg->c_str());
        buf[sizeof(buf) - 1] = '\0';
        Serial.println(buf);
        *msg = buf;
        return false;
    }
    *val = valTmp;
    return true;
}

bool CommandMgr::getDoubleValue(UEvent *event, double *val) {
    String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));

    char *endptr;
    double valTmp = strtod(msg->c_str(), &endptr);
    if (*endptr != '\0') {  // bad integer...
        char buf[200];
        snprintf(buf, sizeof(buf), "Bad floating point value \"%s\"\n", msg->c_str());
        buf[sizeof(buf) - 1] = '\0';
        Serial.println(buf);
        *msg = buf;
        return false;
    }
    *val = valTmp;
    return true;
}

bool CommandMgr::getBoolValue(UEvent *event, bool *val) {
    String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));

    if (strcasecmp(msg->c_str(), "on") == 0 || strcasecmp(msg->c_str(), "1") == 0
            || strcasecmp(msg->c_str(), "y") == 0 || strcasecmp(msg->c_str(), "yes") == 0
            || strcasecmp(msg->c_str(), "true") == 0) {
        *val = true;
        return true;
    } else if (strcasecmp(msg->c_str(), "off") == 0 || strcasecmp(msg->c_str(), "0") == 0
            || strcasecmp(msg->c_str(), "n") == 0 || strcasecmp(msg->c_str(), "no") == 0
            || strcasecmp(msg->c_str(), "false") == 0) {
        *val = false;
        return true;
    } else {
        char buf[200];
        snprintf(buf, sizeof(buf), "Expecting on/yes/y/true/1 or off/no/n/false/0, not \"%s\"\n", msg->c_str());
        buf[sizeof(buf) - 1] = '\0';
        Serial.println(buf);
        *msg = buf;
        return false;
    }
}

bool CommandMgr::getStringValue(UEvent *event, String *val) {
    String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
    *val = *msg;
    val->trim();
    if (val->isEmpty()) {
        *msg = "No value was given";
        return false;
    }
    return true;
}

void CommandMgr::init(UEventLoop *eventLoop) {
    this->eventLoop = eventLoop;
    eventSem = xSemaphoreCreateBinary();
    cmdInternalMenuList = eventLoop->getEventType("cmd:internal", "menuList");
    cmdInternalMenuInfo = eventLoop->getEventType("cmd:internal", "menuInfo");
}

UEventLoop *CommandMgr::getEventLoop() {
    return eventLoop;
}

void CommandMgr::getMenuList(String &menuStr) {
    DynamicJsonBuffer buf;
    JsonArray &menu = buf.createArray();
    for (auto service = serviceCommands.begin(); service != serviceCommands.end(); ++service) {
        menu.add((*service)->serviceName);
    }
    menu.printTo(menuStr);
}

void CommandMgr::getMenuInfo(const char *menuName, String &menuStr) {
    DynamicJsonBuffer buf;
    JsonObject &menu = buf.createObject();
    for (auto service = serviceCommands.begin(); service != serviceCommands.end(); ++service) {
        if (strcmp((*service)->getServiceName(), menuName) == 0) {
            JsonObject &s = buf.createObject();
            for (auto cmd = (*service)->commandEntries.begin(); cmd != (*service)->commandEntries.end(); ++cmd) {
                (*cmd)->addMenuInfo(&s);
            }
            s["status"] = "Show the current status/configuration\n";
            s["save"] = "save [config name]: Save the current config, optionally under a specified config name";
            s["load"] = "load [config name]: Load the specified config, by default the one that was previously loaded, or \"default\" if none";
            s["configs"] = "List the saved configs";
            s["help"] = "List available commands and help";
            menu[(*service)->getServiceName()] = s;
            break;
        }
    }
    menu.printTo(menuStr);
}

/**
 * NOTE: Not reentrant - shouldn't be called while processing another command line.
 */
bool CommandMgr::processCommandLine(const char *channel, String *cmd) {
    if (cmd->length() >= 1024) {
        *cmd = "Command too long, maximum size 1024 bytes";
        return false;
    }
    StringSplitter<1024> splitter(cmd);
    const char *commandClassStr = splitter.nextWord("cmd:");
    const char *commandStr = splitter.nextWord();
    const char *argsStr = splitter.rest();

// Serial.printf("processCommandLine: cmd %s, command class %s, command %s, args \"%s\"\n", cmd->c_str(), commandClassStr, commandStr, argsStr);
// Serial.printf("In CommandMgr::processCommandLine() Heap %d, free %d\n", ESP.getHeapSize(), ESP.getFreeHeap());

    int eventType = eventLoop->findEventType(commandClassStr, commandStr);
    if (eventType == 0) {
        *cmd = "Unrecognized command";
        return false;
    }

    // handle internal commands
    if (eventType == cmdInternalMenuList) {
        *cmd = "@internal:menuList:";
        getMenuList(*cmd);
        return true;
    } else if (eventType == cmdInternalMenuInfo) {
        *cmd = "@internal:menuInfo:";
        getMenuInfo(argsStr, *cmd);
        return true;
    }

    *cmd = argsStr;
    bool isProcessed = false;

    // Serial.printf("CommandMgr received from [%s] command [%d:%d] \"%s\" \"%s\" \"%s\"\n",
    //               channel, eventType >> 16, eventType & 0xFFFF, commandClassStr, commandStr, cmd->c_str());

    mon.enter();  // serialize use of semaphore, in case many threads call this method
// Serial.printf("CommandMgr entered monitor, event loop task: %p, current task: %p\n", eventLoop->getProcessingTask(), xTaskGetCurrentTaskHandle());

    // we're effectively serializing command execution, because we want to be notified
    // for each execution
    {
        if (xTaskGetCurrentTaskHandle() == eventLoop->getProcessingTask()) {
            // We're calling this method from the event loop's processing task, so we
            // cannot wait on a semaphore for the processing signal.
            // Go process the event directly
// Serial.printf("CommandMgr processing event while in event loop's thread\n");
            isProcessed = eventLoop->processEvent(UEvent(eventType, cmd));
// Serial.printf("CommandMgr done processing event\n");
        } else {
            // bool queueEvent(UEvent &event, std::function<void(UEvent*)> finalizer,
            //     std::function<void(UEvent *event, bool isProcessed)> onProcess = nullptr, SemaphoreHandle_t = nullptr);

// Serial.printf("CommandMgr queuing event for processing  while in a thread different from event loop's thread\n");

            UEvent event(eventType, cmd);
            bool queued = eventLoop->queueEvent(
                event,
                [](UEvent *event) -> void {},
                [&isProcessed](UEvent *event, bool inIsProcessed) {
                    // Serial.printf("Event leaving the queue, %s\n", inIsProcessed ? "processed" : "not processed");
                    isProcessed = inIsProcessed;
                    return;
                },
                eventSem);
            if (!queued) {
                // Serial.printf("Event not queued\n");
                *cmd = "Error queueing the command";
                isProcessed = false;
            }

// Serial.printf("CommandMgr waiting on semaphore for event to be processed\n");
            xSemaphoreTake(eventSem, portMAX_DELAY);
// Serial.printf("CommandMgr semaphore taken, event processed\n");
            // we'll have the result in cmd and isProcessed will have been set
        }
    }
    mon.leave();
    // Serial.printf("CommandMgr returning from [%s] command [%d:%d] \"%s\" \"%s\" \"%s\"\n",
    //               channel, eventType >> 16, eventType & 0xFFFF, commandClassStr, commandStr, cmd->c_str());
    return isProcessed;
}

//
// ServiceCommands' data
//

ServiceCommands::ParamData::ParamData(Type type, const char *theName, bool isConstant) : name(theName) {
    this->type = type;
    isPersistent = true;
    includeInStatus = true;
}

ServiceCommands::ParamData::ParamData(Type type, const String *theName) : name(*theName) {
    this->type = type;
    isPersistent = true;
    includeInStatus = true;
}

void ServiceCommands::ParamData::addMenuInfo(JsonObject *menuInfo) {
    if (cmd.get() != nullptr && help.get() != nullptr) {
        (*menuInfo)[cmd.get()] = help.get();
    }
}
void ServiceCommands::ParamData::addHelpInfo(String *msg) {
    if (cmd.get() != nullptr && help.get() != nullptr) {
        msg->concat("    ");
        msg->concat(cmd.get());
        msg->concat(": ");
        msg->concat(help.get());
        msg->concat("\n");
    }
}

ServiceCommands::IntData::IntData(const char *name, bool isConstant) : ParamData(ParamData::Type::INT_DATA, name, isConstant) {
    vMin = INT_MIN;
    vMax = INT_MAX;
    ptr = nullptr;
    setFn = nullptr;
    getFn = nullptr;
    event = -1;
    showAsHex = false;
}

ServiceCommands::IntData::IntData(const String *name) : ParamData(ParamData::Type::INT_DATA, name) {
    vMin = INT_MIN;
    vMax = INT_MAX;
    ptr = nullptr;
    setFn = nullptr;
    getFn = nullptr;
    event = -1;
    showAsHex = false;
}

void ServiceCommands::IntData::addStatusInfo(String *msg) {
    if (getFn != nullptr || ptr != nullptr) {
        msg->concat("    ");
        msg->concat(name.get());
        msg->concat(": ");
        int val;
        if (getFn != nullptr) {
            val = getFn();
        } else {  // (ptr != nullptr)
            val = *ptr;
        }
        if (showAsHex) {
            msg->concat("0x"); msg->concat(String(val, 16));
        } else {
            msg->concat(val);
        }
        msg->concat("\n");
    }
}

void ServiceCommands::IntData::addMenuInfo(JsonObject *menuInfo) {
    if (cmd.get() != nullptr && help.get() != nullptr) {
        (*menuInfo)[cmd.get()] = help.get();
    }
}

bool ServiceCommands::IntData::load(const JsonVariant &val, bool isCheckOnly, String *msg) {
    if (!val.success()) {
        // keep current value
        return true;
    }
    if (setFn == nullptr && ptr == nullptr) {
        *msg = "Cannot set value for \"";
        msg->concat(name);
        msg->concat("\", no set function was provided");
        return false;
    }
    if (!val.is<int>()) {
        *msg = "Expecting an integer for \"";
        msg->concat(name);
        msg->concat("\", but got ");
        val.prettyPrintTo(*msg);
        return false;
    }
    int v = val.as<int>();
    if (v < vMin) {
        *msg = "Expecting an integer > ";
        *msg += vMin;
        *msg += " for \"";
        *msg += name;
        msg->concat("\", but got ");
        val.prettyPrintTo(*msg);
        return false;
    }
    if (v > vMax) {
        *msg = "Expecting an integer < ";
        *msg += vMax;
        *msg += " for \"";
        *msg += name;
        msg->concat("\", but got ");
        val.prettyPrintTo(*msg);
        return false;
    }
    if (!isCheckOnly) {
        if (setFn != nullptr) {
            setFn(val.as<int>(), true, msg);
        } else if (ptr != nullptr) {
            *ptr = val.as<int>();
        }
    }
    return true;
}

void ServiceCommands::IntData::save(JsonBuffer *buf, JsonObject *params) {
    if ((getFn == nullptr && ptr == nullptr) || !isPersistent) {
        // no save
        return;
    }
    int val;
    if (getFn != nullptr) {
        val = getFn();
    } else {
        val = *ptr;
    }
    (*params)[name.get()] = val;
}

ServiceCommands::FloatData::FloatData(const char *name, bool isConstant) : ParamData(ParamData::Type::FLOAT_DATA, name, isConstant) {
    vMin = FLT_MIN;
    vMax = FLT_MAX;
    ptr = nullptr;
    setFn = nullptr;
    getFn = nullptr;
    event = -1;
}

ServiceCommands::FloatData::FloatData(const String *name) : ParamData(ParamData::Type::FLOAT_DATA, name) {
    vMin = FLT_MIN;
    vMax = FLT_MAX;
    ptr = nullptr;
    setFn = nullptr;
    getFn = nullptr;
    event = -1;
}

void ServiceCommands::FloatData::addStatusInfo(String *msg) {
    if (getFn != nullptr || ptr != nullptr) {
        msg->concat("    ");
        msg->concat(name.get());
        msg->concat(": ");
        float val;
        if (getFn != nullptr) {
            val = getFn();
        } else {  // (ptr != nullptr)
            val = *ptr;
        }
        msg->concat(val);
        msg->concat("\n");
    }
}

void ServiceCommands::FloatData::addMenuInfo(JsonObject *menuInfo) {
    if (cmd.get() != nullptr && help.get() != nullptr) {
        (*menuInfo)[cmd.get()] = help.get();
    }
}

bool ServiceCommands::FloatData::load(const JsonVariant &val, bool isCheckOnly, String *msg) {
    if (!val.success()) {
        // keep current value
        return true;
    }
    if (setFn == nullptr && ptr == nullptr) {
        *msg = "Cannot set value for \"";
        msg->concat(name);
        msg->concat("\", no set function was provided");
        return false;
    }
    if (!val.is<float>()) {
        *msg = "Expecting a float for \"";
        msg->concat(name);
        msg->concat("\", but got ");
        val.prettyPrintTo(*msg);
        return false;
    }
    float v = val.as<float>();
    if (v < vMin) {
        *msg = "Expecting a float >= ";
        *msg += vMin;
        *msg += " for \"";
        *msg += name;
        msg->concat("\", but got ");
        val.prettyPrintTo(*msg);
        return false;
    }
    if (v > vMax) {
        *msg = "Expecting a float <= ";
        *msg += vMax;
        *msg += " for \"";
        *msg += name;
        msg->concat("\", but got ");
        val.prettyPrintTo(*msg);
        return false;
    }
    if (!isCheckOnly) {
        if (setFn != nullptr) {
            setFn(v, true, msg);
        } else if (ptr != nullptr) {
            *ptr = v;
        }
    }
    return true;
}

void ServiceCommands::FloatData::save(JsonBuffer *buf, JsonObject *params) {
    if ((getFn == nullptr && ptr == nullptr) || !isPersistent) {
        // no save
        return;
    }
    float val;
    if (getFn != nullptr) {
        val = getFn();
    } else {
        val = *ptr;
    }
    (*params)[name.get()] = val;
}

ServiceCommands::BoolData::BoolData(const char *name, bool isConstant) : ParamData(ParamData::Type::BOOL_DATA, name, isConstant) {
    ptr = nullptr;
    setFn = nullptr;
    getFn = nullptr;
    event = -1;
    eventOn = -1;
    eventOff = -1;
}

ServiceCommands::BoolData::BoolData(String *name) : ParamData(ParamData::Type::BOOL_DATA, name) {
    ptr = nullptr;
    setFn = nullptr;
    getFn = nullptr;
    event = -1;
    eventOn = -1;
    eventOff = -1;
}

void ServiceCommands::BoolData::addMenuInfo(JsonObject *menuInfo) {
    ParamData::addMenuInfo(menuInfo);
    if (this->ptr != nullptr || this->setFn != nullptr) {
        if (cmdOn != nullptr && helpOn != nullptr) {
            (*menuInfo)[cmdOn.get()] = helpOn.get();
        }
        if (cmdOff != nullptr && helpOff != nullptr) {
            (*menuInfo)[cmdOff.get()] = helpOff.get();
        }
        if ( cmd.get() != nullptr && help.get() != nullptr) {
            (*menuInfo)[cmd.get()] = help.get();
        }
    }
}

void ServiceCommands::BoolData::addHelpInfo(String *msg) {
    ParamData::addHelpInfo(msg);
    int cntCmdOnOff = 0;
    if (cmdOn != nullptr && helpOn != nullptr) {
        ++cntCmdOnOff;
        msg->concat("    ");
        msg->concat(cmdOn.get());
        msg->concat(": ");
        msg->concat(helpOn.get());
        msg->concat("\n");
    }
    if (cmdOff != nullptr && helpOff != nullptr) {
        ++cntCmdOnOff;
        msg->concat("    ");
        msg->concat(cmdOff);
        msg->concat(": ");
        msg->concat(helpOff);
        msg->concat("\n");
    }
    if (cmd != nullptr && help != nullptr && cntCmdOnOff < 2) {
        msg->concat("    ");
        msg->concat(cmd);
        msg->concat(": ");
        msg->concat(help);
        msg->concat("\n");
    }
}

void ServiceCommands::BoolData::addStatusInfo(String *msg) {
    if (getFn != nullptr || ptr != nullptr) {
        msg->concat("    ");
        msg->concat(name.get());
        msg->concat(": ");
        bool val;
        if (getFn != nullptr) {
            val = getFn();
        } else {  // (ptr != nullptr)
            val = *ptr;
        }
        msg->concat(val ? "on\n" : "off\n");
    }
}

bool ServiceCommands::BoolData::load(const JsonVariant &val, bool isCheckOnly, String *msg) {
    if (!val.success()) {
        // keep current value
        return true;
    }
    if (setFn == nullptr && ptr == nullptr) {
        *msg = "Cannot set value for \"";
        msg->concat(name);
        msg->concat("\", no set function was provided");
        return false;
    }
    if (!val.is<bool>()) {
        *msg = "Expecting a boolean for \"";
        msg->concat(name);
        msg->concat("\", but got ");
        val.prettyPrintTo(*msg);
        return false;
    }
    if (!isCheckOnly) {
        if (setFn != nullptr) {
            setFn(val.as<bool>(), true, msg);
        } else if (ptr != nullptr) {
            *ptr = val.as<bool>();
        }
    }
    return true;
}

void ServiceCommands::BoolData::save(JsonBuffer *buf, JsonObject *params) {
    if ((getFn == nullptr && ptr == nullptr) || !isPersistent) {
        // no save
        return;
    }
    bool val;
    if (getFn != nullptr) {
        val = getFn();
    } else {
        val = *ptr;
    }
    (*params)[name.get()] = val;
}

ServiceCommands::StringData::StringData(const char *name, bool isConstant) : ParamData(ParamData::Type::STRING_DATA, name, isConstant) {
    ptr = nullptr;
    setFn = nullptr;
    getFn = nullptr;
    event = -1;
}

ServiceCommands::StringData::StringData(const String *name) : ParamData(ParamData::Type::STRING_DATA, name) {
    ptr = nullptr;
    setFn = nullptr;
    getFn = nullptr;
    event = -1;
}

void ServiceCommands::StringData::addStatusInfo(String *msg) {
    if (getFn != nullptr || ptr != nullptr) {
        msg->concat("    ");
        msg->concat(name.get());
        msg->concat(": ");
        String val;
        if (getFn != nullptr) {
            getFn(&val);
        } else {  // (ptr != nullptr)
            val = *ptr;
        }
        msg->concat(val);
        msg->concat("\n");
    }
}

void ServiceCommands::StringData::addMenuInfo(JsonObject *menuInfo) {
    if (cmd.get() != nullptr && help.get() != nullptr) {
        (*menuInfo)[cmd.get()] = help.get();
    }
}

bool ServiceCommands::StringData::load(const JsonVariant &val, bool isCheckOnly, String *msg) {
    if (!val.success()) {
        // keep current value
        return true;
    }
    if (setFn == nullptr && ptr == nullptr) {
        *msg = "Cannot set value for \"";
        msg->concat(name);
        msg->concat("\", no set function was provided");
        return false;
    }
    if (!val.is<const char *>()) {
        *msg = "Expecting a string for \"";
        msg->concat(name);
        msg->concat("\", but got ");
        val.prettyPrintTo(*msg);
        return false;
    }
    if (!isCheckOnly) {
        if (setFn != nullptr) {
            setFn(String(val.as<const char *>()), true, msg);
        } else if (ptr != nullptr) {
            *ptr = String(val.as<const char *>());
        }
    }
    return true;
}

void ServiceCommands::StringData::save(JsonBuffer *buf, JsonObject *params) {
    if ((getFn == nullptr && ptr == nullptr) || !isPersistent) {
        // no save
        return;
    }
    String val;
    if (getFn != nullptr) {
        getFn(&val);
    } else {
        val = *ptr;
    }
    (*params)[name.get()] = val;
}

ServiceCommands::SysPinData::SysPinData(const char *name, bool isConstant) : ParamData(ParamData::Type::PIN_DATA, name, isConstant) {
    ptr = nullptr;
    setFn = nullptr;
    getFn = nullptr;
    event = -1;
}

ServiceCommands::SysPinData::SysPinData(const String *name) : ParamData(ParamData::Type::PIN_DATA, name) {
    ptr = nullptr;
    setFn = nullptr;
    getFn = nullptr;
    event = -1;
}

void ServiceCommands::SysPinData::addStatusInfo(String *msg) {
    if (getFn != nullptr || ptr != nullptr) {
        msg->concat("    ");
        msg->concat(name.get());
        msg->concat(": ");
        msg->concat(ptr->getPin());
        msg->concat("\n");
    }
}

void ServiceCommands::SysPinData::addMenuInfo(JsonObject *menuInfo) {
    if (cmd.get() != nullptr && help.get() != nullptr) {
        (*menuInfo)[cmd.get()] = help.get();
    }
}

bool ServiceCommands::SysPinData::load(const JsonVariant &val, bool isCheckOnly, String *msg) {
    if (!val.success()) {
        // keep current value
        return true;
    }
    if (!val.is<int>()) {
        *msg = "Expecting an integer for \"";
        msg->concat(name);
        msg->concat("\", but got ");
        val.prettyPrintTo(*msg);
        return false;
    }
    int v = val.as<int>();
    if (v < -1) {
        *msg = "Expecting an integer >0 or -1 for \"";
        *msg += name;
        msg->concat("\", but got ");
        val.prettyPrintTo(*msg);
        return false;
    }
    if (v > 99) {
        *msg = "Expecting an integer < 99 for \"";
        *msg += name;
        msg->concat("\", but got ");
        val.prettyPrintTo(*msg);
        return false;
    }
    if (!isCheckOnly) {
        if (setFn != nullptr) {
            setFn(val.as<int>(), true, msg);
        } else if (ptr != nullptr) {
            ptr->setPin(val.as<int>());
        }
    }
    return true;
}

void ServiceCommands::SysPinData::save(JsonBuffer *buf, JsonObject *params) {
    if (!isPersistent) {
        // no save
        return;
    }
    (*params)[name.get()] = ptr->getPin();
}

ServiceCommands::JsonData::JsonData(const char *name, bool isConstant) : ParamData(ParamData::Type::JSON_DATA, name, isConstant) {
    setFn = nullptr;
    getFn = nullptr;
    event = -1;
}

ServiceCommands::JsonData::JsonData(const String *name) : ParamData(ParamData::Type::JSON_DATA, name) {
    setFn = nullptr;
    getFn = nullptr;
    event = -1;
}

void ServiceCommands::JsonData::addStatusInfo(String *msg) {
    if (getFn != nullptr) {
        msg->concat("    ");
        msg->concat(name.get());
        msg->concat(": ");
        if (getFn != nullptr) {
            DynamicJsonBuffer buf;
            JsonVariant val = getFn(buf);
            val.prettyPrintTo(*msg);
        }
        msg->concat("\n");
    }
}

void ServiceCommands::JsonData::addMenuInfo(JsonObject *menuInfo) {
    if (cmd.get() != nullptr && help.get() != nullptr) {
        (*menuInfo)[cmd.get()] = help.get();
    }
}

bool ServiceCommands::JsonData::load(const JsonVariant &val, bool isCheckOnly, String *msg) {
    if (!val.success()) {
        // keep current value
        return true;
    }
    if (setFn == nullptr) {
        *msg = "Cannot set value for \"";
        msg->concat(name);
        msg->concat("\", no set function was provided");
        return false;
    }
    if (!isCheckOnly) {
        if (setFn != nullptr) {
            setFn(val, true, msg);
        }
    }
    return true;
}

void ServiceCommands::JsonData::save(JsonBuffer *buf, JsonObject *params) {
    if ((getFn == nullptr) || !isPersistent) {
        // no save
        return;
    }
    String val;
    if (getFn != nullptr) {
        JsonVariant val = getFn(*buf);
        (*params)[name.get()] = val;
    }
}

//  .d8888b.                            d8b                   .d8888b.                                                              888          
// d88P  Y88b                           Y8P                  d88P  Y88b                                                             888          
// Y88b.                                                     888    888                                                             888          
//  "Y888b.    .d88b.  888d888 888  888 888  .d8888b .d88b.  888         .d88b.  88888b.d88b.  88888b.d88b.   8888b.  88888b.   .d88888 .d8888b  
//     "Y88b. d8P  Y8b 888P"   888  888 888 d88P"   d8P  Y8b 888        d88""88b 888 "888 "88b 888 "888 "88b     "88b 888 "88b d88" 888 88K      
//       "888 88888888 888     Y88  88P 888 888     88888888 888    888 888  888 888  888  888 888  888  888 .d888888 888  888 888  888 "Y8888b. 
// Y88b  d88P Y8b.     888      Y8bd8P  888 Y88b.   Y8b.     Y88b  d88P Y88..88P 888  888  888 888  888  888 888  888 888  888 Y88b 888      X88 
//  "Y8888P"   "Y8888  888       Y88P   888  "Y8888P "Y8888   "Y8888P"   "Y88P"  888  888  888 888  888  888 "Y888888 888  888  "Y88888  88888P' 

ServiceCommands *CommandMgr::getServiceCommands(const char *serviceName) {
    for (auto i = serviceCommands.begin(); i != serviceCommands.end(); i++) {
        if ((*i)->serviceName.equals(serviceName)) {
            return *i;
        }
    }
    // else create a new one
    ServiceCommands *s = new ServiceCommands(serviceName, this);

    serviceCommands.push_back(s);
    return s;
}

ServiceCommands::ServiceCommands(const char *serviceName, CommandMgr *cmdMgr) {
    this->serviceName = serviceName;
    this->cmdMgr = cmdMgr;
    beforeLoadFn = nullptr;
    afterLoadFn = nullptr;
    beforeSaveFn = nullptr;
    afterSaveFn = nullptr;

    String sn;
    sn.concat("cmd:");
    sn.concat(serviceName);
    eventClass = cmdMgr->getEventLoop()->getEventClassType(sn.c_str());

    uint32_t helpEvent = cmdMgr->getEventLoop()->getEventType(sn.c_str(), "help");
    cmdMgr->getEventLoop()->onEvent(helpEvent, [this](UEvent *event) -> bool {
        String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
        *msg = "Help for service ";
        msg->concat(this->serviceName);
        msg->concat("\n");
        for (auto c = commandEntries.begin(); c != commandEntries.end(); ++c) {
            (*c)->addHelpInfo(msg);
        }
        msg->concat("    status --> Show the current status/configuration\n");
        msg->concat("    save [config name] --> Saves the current config, optionally under a specified config name\n");
        msg->concat("    load [config name] --> Loads the specified config, by default the one that was previously loaded, or \"default\" if none.\n");
        msg->concat("    configs --> Lists the saved configs\n");
        return true;
    });

    uint32_t statusEvent = cmdMgr->getEventLoop()->getEventType(sn.c_str(), "status");
    cmdMgr->getEventLoop()->onEvent(statusEvent, [this](UEvent *event) -> bool {
        String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));

        if (beforeStatusFn != nullptr) {
            bool rc = beforeStatusFn(msg);
            if (rc) {
                return true;
            }
        }
        *msg = "Status for service ";
        msg->concat(this->serviceName);
        msg->concat("\n");
        for (auto c = commandEntries.begin(); c != commandEntries.end(); ++c) {
            if ((*c)->includeInStatus) {
                (*c)->addStatusInfo(msg);
            } else {
                msg->concat("    ");
                msg->concat((*c)->name.get());
                msg->concat(": <not shown>\n");
            }
        }
        if (afterStatusFn != nullptr) {
            afterStatusFn(msg);
        }

        return true;
    });

    uint32_t configsEvent = cmdMgr->getEventLoop()->getEventType(sn.c_str(), "configs");
    cmdMgr->getEventLoop()->onEvent(configsEvent, [this](UEvent *event) -> bool {
        String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
        bool rc;
        rc = this->listKeyNames(msg);
        return rc;
    });

    uint32_t loadEvent = cmdMgr->getEventLoop()->getEventType(sn.c_str(), "load");
    cmdMgr->getEventLoop()->onEvent(loadEvent, [this](UEvent *event) -> bool {
        String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
        bool rc;
        String keyName;
        if (msg->isEmpty()) {
            keyName = "default";
            rc = this->load(keyName.c_str(), msg);
        } else {
            keyName = *msg;
            keyName.trim();
            rc = this->load(keyName.c_str(), msg);
        }
        if (rc && currentKeyName != keyName) {
            currentKeyName = keyName;
        }
        return true;
    });

    uint32_t saveEvent = cmdMgr->getEventLoop()->getEventType(sn.c_str(), "save");
    cmdMgr->getEventLoop()->onEvent(saveEvent, [this](UEvent *event) -> bool {
        String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
        bool rc;
        String keyName;
        if (msg->isEmpty()) {
            if (currentKeyName.isEmpty()) {
                keyName = "default";
            } else {
                keyName = currentKeyName;
            }
            rc = this->save(keyName.c_str(), msg);
        } else {
            keyName = *msg;
            keyName.trim();
            rc = this->save(keyName.c_str(), msg);
        }
        if (rc && currentKeyName != keyName) {
            currentKeyName = keyName;
        }
        return true;
    });
}

const char *ServiceCommands::getServiceName() {
    return serviceName.c_str();
}

void ServiceCommands::registerIntData(IntDataBuilder &builder) {
    IntData *data = builder.getData();
    if (data->cmd != nullptr) {
        String sn("cmd:");
        sn.concat(serviceName);
        data->event = cmdMgr->getEventLoop()->getEventType(sn.c_str(), data->cmd);
        if (data->setFn != nullptr || data->getFn != nullptr || data->ptr != nullptr) {
            cmdMgr->getEventLoop()->onEvent(data->event, [data](UEvent *event) -> bool {
                String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
                const char *p = msg->c_str();
                while (*p != '\0' && isspace(*p)) {
                    ++p;
                }
                if (*p == '\0') {  // doing a show value
                    msg->clear();
                    int val;
                    if (data->getFn != nullptr) {
                        val = data->getFn();
                        if (data->showAsHex) {
                            msg->concat("0x"); msg->concat(String(val, 16));
                        } else {
                            msg->concat(val);
                        }
                        return true;
                    } else if (data->ptr != nullptr) {
                        val = *data->ptr;
                        if (data->showAsHex) {
                            msg->concat("0x"); msg->concat(String(val, 16));
                        } else {
                            msg->concat(val);
                        }
                        return true;
                    } else {
                        return false;
                    }
                } else {  // doing a set value
                    int val;
                    bool rc = CommandMgr::getIntValue(event, &val);
                    if (!rc) {
                        return false;
                    }
                    // controls
                    if (val < data->vMin) {
                        *msg = "Value ";
                        msg->concat(val);
                        msg->concat(" must be not less than ");
                        msg->concat(data->vMin);
                        return false;
                    }
                    if (val > data->vMax) {
                        *msg = "Value ";
                        msg->concat(val);
                        msg->concat(" must be not greater than ");
                        msg->concat(data->vMax);
                        return false;
                    }
                    if (data->setFn != nullptr) {
                        rc = data->setFn(val, false, msg);
                    } else if (data->ptr) {
                        *data->ptr = val;
                        rc = true;
                    } else {
                        rc = false;
                    }
                    return rc;
                }
            });
        };
    }

    commandEntries.push_back(data);
}

void ServiceCommands::registerFloatData(FloatDataBuilder &builder) {
    FloatData *data = (FloatData *)builder.getData();
    if (data->cmd != nullptr) {
        String sn("cmd:");
        sn.concat(serviceName);
        data->event = cmdMgr->getEventLoop()->getEventType(sn.c_str(), data->cmd);
        if (data->setFn != nullptr || data->getFn != nullptr|| data->ptr != nullptr) {
            cmdMgr->getEventLoop()->onEvent(data->event, [data](UEvent *event) -> bool {
                String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
                const char *p = msg->c_str();
                while (*p != '\0' && isspace(*p)) {
                    ++p;
                }
                if (*p == '\0') {  // doing a show value
                    msg->clear();
                    float val;
                    if (data->getFn != nullptr) {
                        val = data->getFn();
                        msg->concat(val);
                        return true;
                    } else if (data->ptr != nullptr) {
                        val = *data->ptr;
                        msg->concat(val);
                        return true;
                    } else {
                        return false;
                    }
                } else {  // doing a set value
                    float val;
                    bool rc = CommandMgr::getFloatValue(event, &val);
                    if (!rc) {
                        return false;
                    }
                    // controls
                    if (val < data->vMin) {
                        *msg = "Value ";
                        msg->concat(val);
                        msg->concat(" must be not less than ");
                        msg->concat(data->vMin);
                        return false;
                    }
                    if (val > data->vMax) {
                        *msg = "Value ";
                        msg->concat(val);
                        msg->concat(" must be not greater than ");
                        msg->concat(data->vMax);
                        return false;
                    }
                    if (data->setFn != nullptr) {
                        rc = data->setFn(val, false, msg);
                    } else if (data->ptr != nullptr) {
                        *data->ptr = val;
                        rc = true;
                    } else {
                        rc = false;
                    }
                    return rc;
                }
            });
        };
    }

    commandEntries.push_back(data);
}

void ServiceCommands::registerBoolData(BoolDataBuilder &builder) {
    BoolData *data = (BoolData *)builder.getData();
    String sn("cmd:");
    sn.concat(serviceName);
    if (data->cmd != nullptr) {
        data->event = cmdMgr->getEventLoop()->getEventType(sn.c_str(), data->cmd);
        if (data->setFn != nullptr || data->getFn != nullptr || data->ptr != nullptr) {
            cmdMgr->getEventLoop()->onEvent(data->event, [data](UEvent *event) -> bool {
                String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
                const char *p = msg->c_str();
                while (*p != '\0' && isspace(*p)) {
                    ++p;
                }
                if (*p == '\0') {  // doing a show value
                    msg->clear();
                    bool val;
                    if (data->getFn != nullptr) {
                        val = data->getFn();
                        msg->concat(val ? "on" : "off");
                        return true;
                    } else if (data->ptr != nullptr) {
                        val = *data->ptr;
                        msg->concat(val ? "on" : "off");
                        return true;
                    } else {
                        return false;
                    }
                } else {  // doing a set value
                    bool val;
                    bool rc = CommandMgr::getBoolValue(event, &val);
                    if (!rc) {
                        return false;
                    }
                    if (data->setFn != nullptr) {
                        rc = data->setFn(val, false, msg);
                    } else if (data->ptr != nullptr) {
                        *data->ptr = val;
                        rc = true;
                    } else {
                        rc = false;
                    }
                    return rc;
                }
            });
        }
    }

    if (data->cmdOn != nullptr) {
        data->eventOn = cmdMgr->getEventLoop()->getEventType(sn.c_str(), data->cmdOn);
        if (data->setFn != nullptr || data->ptr != nullptr) {
            cmdMgr->getEventLoop()->onEvent(data->eventOn, [data](UEvent *event) -> bool {
                String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
                msg->clear();
                bool rc;
                if (data->setFn != nullptr) {
                    rc = data->setFn(true, false, msg);
                } else {
                    *data->ptr = true;
                    rc = true;
                }
                return rc;
            });
        }
    }

    if (data->cmdOff != nullptr) {
        data->eventOff = cmdMgr->getEventLoop()->getEventType(sn.c_str(), data->cmdOff);
        if (data->setFn != nullptr || data->ptr != nullptr) {
            cmdMgr->getEventLoop()->onEvent(data->eventOff, [data](UEvent *event) -> bool {
                String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
                msg->clear();
                bool rc;
                if (data->setFn != nullptr) {
                    rc = data->setFn(false, false, msg);
                } else {
                    *data->ptr = false;
                    rc = true;
                }
                return rc;
            });
        }
    }

    commandEntries.push_back(data);
}

void ServiceCommands::registerStringData(StringDataBuilder &builder) {
    StringData *data = (StringData *)builder.getData();
    if (data->cmd != nullptr) {
        String sn("cmd:");
        sn.concat(serviceName);
        data->event = cmdMgr->getEventLoop()->getEventType(sn.c_str(), data->cmd);
        if (data->getFn != nullptr || data->setFn != nullptr || data->ptr != nullptr) {
            cmdMgr->getEventLoop()->onEvent(data->event, [data](UEvent *event) -> bool {
                String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
                const char *p = msg->c_str();
                while (*p != '\0' && isspace(*p)) {
                    ++p;
                }
                if (*p == '\0') {  // doing a show value
                    msg->clear();
                    if (data->getFn != nullptr) {
                        String val;
                        data->getFn(&val);
                        msg->concat(val);
                        return true;
                    } else if (data->ptr != nullptr) {
                        msg->concat(*data->ptr);
                        return true;
                    } else {
                        return false;
                    }
                } else {  // doing a set value
                    String val;
                    bool rc = CommandMgr::getStringValue(event, &val);
                    if (!rc) {
                        return false;
                    }
                    if (data->setFn != nullptr) {
                        rc = data->setFn(val, false, msg);
                    } else if (data->ptr != nullptr) {
                        *data->ptr = val;
                        rc = true;
                    } else {
                        rc = false;
                    }
                    return rc;
                }
            });
        };
    }

    commandEntries.push_back(data);
}

void ServiceCommands::registerSysPinData(SysPinDataBuilder &builder) {
    SysPinData *data = builder.getData();
    if (data->cmd != nullptr) {
        String sn("cmd:");
        sn.concat(serviceName);
        data->event = cmdMgr->getEventLoop()->getEventType(sn.c_str(), data->cmd);
        if (data->setFn != nullptr || data->getFn != nullptr || data->ptr != nullptr) {
            cmdMgr->getEventLoop()->onEvent(data->event, [data](UEvent *event) -> bool {
                String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
                const char *p = msg->c_str();
                while (*p != '\0' && isspace(*p)) {
                    ++p;
                }
                if (*p == '\0') {  // doing a show value
                    msg->clear();
                    msg->concat(*data->ptr);
                    return true;
                } else {  // doing a set value
                    int val;
                    bool rc = CommandMgr::getIntValue(event, &val);
                    if (!rc) {
                        return false;
                    }
                    // controls
                    if (val < -1) {
                        *msg = "Value ";
                        msg->concat(val);
                        msg->concat(" must be positive or -1");
                        return false;
                    }
                    if (val > 99) {
                        *msg = "Value ";
                        msg->concat(val);
                        msg->concat(" must be not greater than 99");
                        return false;
                    }
                    if (data->setFn != nullptr) {
                        rc = data->setFn(val, false, msg);
                    } else if (data->ptr) {
                        data->ptr->setPin(val);
                        rc = true;
                    } else {
                        rc = false;
                    }
                    return rc;
                }
            });
        };
    }

    commandEntries.push_back(data);
}

void ServiceCommands::registerJsonData(JsonDataBuilder &builder) {
    JsonData *data = (JsonData *)builder.getData();
    if (data->cmd != nullptr) {
        String sn("cmd:");
        sn.concat(serviceName);
        data->event = cmdMgr->getEventLoop()->getEventType(sn.c_str(), data->cmd);
        if (data->getFn != nullptr || data->setFn != nullptr) {
            cmdMgr->getEventLoop()->onEvent(data->event, [data](UEvent *event) -> bool {
                String *msg = reinterpret_cast<String *>(const_cast<void *>(event->dataPtr));
                const char *p = msg->c_str();
                while (*p != '\0' && isspace(*p)) {
                    ++p;
                }
                if (*p == '\0') {  // doing a show value
                    msg->clear();
                    if (data->getFn != nullptr) {
                        DynamicJsonBuffer buf;
                        JsonVariant val = data->getFn(buf);
                        val.prettyPrintTo(*msg);
                        return true;
                    } else {
                        return false;
                    }
                } else {  // doing a set value
                    String val;
                    bool rc = CommandMgr::getStringValue(event, &val);
                    if (!rc) {
                        return false;
                    }
                    if (data->setFn != nullptr) {
                        DynamicJsonBuffer buf;
                        JsonVariant json = buf.parse(val);
                        rc = data->setFn(json, false, msg);
                    } else {
                        rc = false;
                    }
                    return rc;
                }
            });
        };
    }

    commandEntries.push_back(data);
}

void ServiceCommands::onBeforeStatus(std::function<bool(String *msg)> beforeStatusFn) {
    this->beforeStatusFn = beforeStatusFn;
}

void ServiceCommands::onAfterStatus(std::function<void(String *msg)> afterStatusFn) {
    this->afterStatusFn = afterStatusFn;
}

void ServiceCommands::getCurrentKeyName(String *keyName) {
    *keyName = currentKeyName;
}

bool ServiceCommands::save(const char *keyName, String *msg) {
    bool rc;
    if (beforeSaveFn != nullptr) {
        rc = beforeSaveFn(msg);
        if (!rc) {
            return false;
        }
    }
    DynamicJsonBuffer buf;
    JsonObject *params;
    Serial.printf("Saving config %s/%s\n", serviceName.c_str(), keyName);
    rc = readConfigFile(buf, &params, msg);
    if (!rc) {
        Serial.printf("Error reading config file: for %s: %s", serviceName.c_str(), msg->c_str());
        return false;
    }
    params->remove(keyName);
    JsonObject &entry = params->createNestedObject(keyName);
    for (auto c = commandEntries.begin(); c != commandEntries.end(); ++c) {
        if ((*c)->isPersistent) {
            (*c)->save(&buf, &entry);
        }
    }
    params->remove("defaultKey");  // so that it always goes to the end of the config file
    (*params)["defaultKey"] = keyName;
    String configFile;
    configFile.concat("/");
    configFile.concat(serviceName);
    configFile.concat(".conf.json");
    File f = SPIFFS.open(configFile, "w");
    params->prettyPrintTo(f);
    f.close();
    Serial.printf("Saved config %s/%s\n", serviceName.c_str(), keyName);
    if (afterSaveFn != nullptr) {
        afterSaveFn(msg);
    }

    return true;
}

bool ServiceCommands::listKeyNames(String *msg) {
    DynamicJsonBuffer buf;
    JsonObject *params;
    bool rc = readConfigFile(buf, &params, msg);
    if (!rc) {
        return false;
    }
    *msg = "Available configurations for ";
    msg->concat(serviceName);
    msg->concat("\n");
    for (auto p = params->begin(); p != params->end(); ++p) {
        if (p->value.is<JsonObject>()) {
            msg->concat("    ");
            msg->concat((*p).key);
            msg->concat("\n");
        }
    }
    const char *keyName = (*params)["defaultKey"];
    if (keyName == nullptr || keyName[0] == '\0') {
        if ((*params)["default"].is<JsonObject>()) {
            msg->concat("Default configuration: \"default\"\n");
        } else {
            msg->concat("Default configuration: not defined\n");
        }
    } else {
        msg->concat("Default configuration: ");
        msg->concat(keyName);
        msg->concat("\n");
    }
    msg->concat("Current configuration: ");
    msg->concat(currentKeyName);
    msg->concat("\n");
    return true;
}

bool ServiceCommands::readConfigFile(DynamicJsonBuffer &buf, JsonObject **params, String *msg) {
    String configFile;
    configFile.concat("/");
    configFile.concat(serviceName);
    configFile.concat(".conf.json");
    if (!SPIFFS.exists(configFile)) {
        // empty params
        *params = &buf.createObject();
        return true;
    }
    File f = SPIFFS.open(configFile, "r");
    JsonObject &cfg = buf.parseObject(f);
    f.close();
    if (!cfg.success()) {
        Serial.printf("Error reading \"%s\", file ignored", configFile.c_str());
        return false;
    }
    Serial.printf("Read configuration for service %s from file %s:\n", serviceName.c_str(), configFile.c_str());
    *params = &cfg;
    return true;
}

bool ServiceCommands::load(const char *keyName, String *msg) {
    bool rc;
    if (beforeLoadFn != nullptr) {
        rc = beforeLoadFn(msg);
        if (!rc) {
            return false;
        }
    }
    DynamicJsonBuffer buf;
    JsonObject *params;
    Serial.printf("Loading config %s/%s\n", serviceName.c_str(), keyName == nullptr ? "<default>" : keyName);
    rc = readConfigFile(buf, &params, msg);
    if (!rc) {
        Serial.printf("Error loading config %s: %s\n", serviceName.c_str(), msg->c_str());
        return false;
    }
    if (keyName == nullptr || keyName[0] == '\0') {
        keyName = (*params)["defaultKey"];
        if (keyName == nullptr || keyName[0] == '\0') {
            keyName = "default";
        }
    }
    String msg2;
    bool isOk = true;
    for (auto c = commandEntries.begin(); isOk && c != commandEntries.end(); ++c) {
        if ((*c)->isPersistent) {
            const JsonVariant &val = (*params)[keyName][(*c)->name.get()];
            isOk = (*c)->load(val, true, &msg2);
            if (!isOk) {
                String configFile;
                configFile.concat("/");
                configFile.concat(serviceName);
                configFile.concat(".conf.json");
                Serial.printf("Error loading \"%s/%s\" : %s\n", configFile.c_str(), keyName, msg2.c_str());
                msg->concat(msg2);
                msg->concat("\n");
            }
        }
    }
    if (!isOk) {
        Serial.printf("Error loading data from config %s/%s: %s\n", serviceName.c_str(), keyName, msg->c_str());
        return false;
    }
    for (auto c = commandEntries.begin(); isOk && c != commandEntries.end(); ++c) {
        if ((*c)->isPersistent) {
            const JsonVariant &val = (*params)[keyName][(*c)->name.get()];
            (*c)->load(val, false, &msg2);
        }
    }
    if (afterLoadFn != nullptr) {
        afterLoadFn(msg);
    }
    Serial.printf("Loaded config %s/%s\n", serviceName.c_str(), keyName);
    currentKeyName = keyName;
    return true;
}
