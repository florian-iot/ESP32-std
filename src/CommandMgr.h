#ifndef INCL_COMMAND_MGR_H
#define INCL_COMMAND_MGR_H

#include <float.h>
#include <vector>
#include <map>
#include <unordered_map>
#include <WString.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "System.h"

class StaticString {
private:
  const char *val;
public:
  explicit StaticString(const char *v) : val(v) { }
  StaticString(const StaticString &other) : val(other.val) { }
  const char *get() { return val; }
};

class ConstString {
private:
  String valString;
  const char *val;
public:
  ConstString(): valString((const char *)nullptr), val(nullptr) {}
  explicit ConstString(const String &str): valString(str), val(nullptr) {}
  explicit ConstString(StaticString str): valString((const char *)nullptr), val(str.get()) {}
  explicit ConstString(const ConstString &other) { valString = other.valString; val = other.val; }
  ~ConstString() { }
  const char *get() { return valString ? valString.c_str() : val; }
  operator const char *() { return get(); }
  void set(const ConstString &str) { valString = str.valString; val = str.val; }
  void set(const String &str) { valString = str; val = nullptr; }
  void set(const char *str, bool isConstant) { valString = (const char *)nullptr; val = str; }
};

class CommandMgr;

/**
 * Allows loading from JSON doc, saving to JSON doc.
 * Registers commands, used to change data values or just as commands with user-defined functions.
 */
class ServiceCommands {
  friend class CommandMgr;
public:

  /**
   * name is the data name, same as json name if data can be loaded/saved (isPersistent).
   * cmd may be null, in which case no command will be available.
   * If setFn or getFn are null, then ptr is used to set/get the value, and it must not be null.
   */
  struct ParamData {
    enum Type {
      INT_DATA, FLOAT_DATA, BOOL_DATA, STRING_DATA, JSON_DATA, PIN_DATA
    };
    Type type;
    ConstString name;
    bool isPersistent;
    ConstString cmd;
    ConstString help;
    bool includeInStatus;
    ParamData(Type type, const char *theName, bool isConstant);
    ParamData(Type type, const String *theName);
    virtual void addMenuInfo(JsonObject *menuInfo);
    virtual void addHelpInfo(String *msg);
    virtual void addStatusInfo(String *msg) = 0;
    virtual bool load(const JsonVariant &val, bool isCheckOnly, String *msg) = 0;
    virtual void save(JsonBuffer *buf, JsonObject *params) = 0;
  };

  struct IntData: public ParamData {
    int vMin;
    int vMax;
    int *ptr;
    bool showAsHex;
    std::function<bool(int val, bool isLoading, String *msg)> setFn;
    std::function<int()> getFn;
    uint32_t event;
    IntData(const char *name, bool isConstant);
    IntData(const String *name);
    virtual void addStatusInfo(String *msg);
    virtual void addMenuInfo(JsonObject *menuInfo);
    virtual bool load(const JsonVariant &val, bool isCheckOnly, String *msg);
    virtual void save(JsonBuffer *buf, JsonObject *params);
  };

  class IntDataBuilder {
  private:
    IntData *data;
  public:
    IntDataBuilder(const char *name, bool isConstant) {
      data = new IntData(name, isConstant);
    }
    IntDataBuilder(const String *name) {
      data = new IntData(name);
    }

    IntDataBuilder &cmd(const char *cmd, bool isConstant) { data->cmd.set(cmd, isConstant); return *this; }
    IntDataBuilder &cmd(const String &cmd) { data->cmd.set(cmd); return *this; }
    IntDataBuilder &isPersistent(bool p) { data->isPersistent = p; return *this; }
    IntDataBuilder &help(const char *h, bool isConstant) { data->help.set(h, isConstant); return *this; }
    IntDataBuilder &help(const String h) { data->help.set(h); return *this; }
    IntDataBuilder &includeInStatus(bool b) { data->includeInStatus = b; return *this; }

    IntDataBuilder &vMin(int val) { data->vMin = val; return *this; }
    IntDataBuilder &vMax(int val) { data->vMax = val; return *this; }
    IntDataBuilder &isShowAsHex(bool val = true) { data->showAsHex = val; return *this; }
    IntDataBuilder &ptr(int *ptr) { data->ptr = ptr; return *this; }
    IntDataBuilder &setFn(std::function<bool(int val, bool isLoading, String *msg)> fn) { data->setFn = fn; return *this; }
    IntDataBuilder &getFn(std::function<int()> fn) { data->getFn = fn; return *this; }

    IntData *getData() const { return data; }
  };

  struct FloatData: public ParamData {
    float vMin;
    float vMax;
    float *ptr;
    std::function<bool(float val, bool isLoading, String *msg)> setFn;
    std::function<float()> getFn;
    uint32_t event;
    FloatData(const char *name, bool isConstant);
    FloatData(const String *name);
    virtual void addStatusInfo(String *msg);
    virtual void addMenuInfo(JsonObject *menuInfo);
    virtual bool load(const JsonVariant &val, bool isCheckOnly, String *msg);
    virtual void save(JsonBuffer *buf, JsonObject *params);
  };

  class FloatDataBuilder {
  private:
    FloatData *data;
  public:
    FloatDataBuilder(const char *name, bool isConstant) {
      data = new FloatData(name, isConstant);
    }
    FloatDataBuilder(const String *name) {
      data = new FloatData(name);
    }

    FloatDataBuilder &cmd(const char *cmd, bool isConstant) { data->cmd.set(cmd, isConstant); return *this; }
    FloatDataBuilder &cmd(const String &cmd) { data->cmd.set(cmd); return *this; }
    FloatDataBuilder &isPersistent(bool p) { data->isPersistent = p; return *this; }
    FloatDataBuilder &help(const char *h, bool isConstant) { data->help.set(h, isConstant); return *this; }
    FloatDataBuilder &help(const String h) { data->help.set(h); return *this; }
    FloatDataBuilder &includeInStatus(bool b) { data->includeInStatus = b; return *this; }

    FloatDataBuilder &vMin(float val) { data->vMin = val; return *this; }
    FloatDataBuilder &vMax(float val) { data->vMax = val; return *this; }
    FloatDataBuilder &ptr(float *ptr) { data->ptr = ptr; return *this; }
    FloatDataBuilder &setFn(std::function<bool(float val, bool isLoading, String *msg)> fn) { data->setFn = fn; return *this; }
    FloatDataBuilder &getFn(std::function<float()> fn) { data->getFn = fn; return *this; }

    FloatData *getData() const { return data; }
  };

  struct BoolData: public ParamData {
    ConstString cmdOn;
    ConstString cmdOff;
    ConstString helpOn;
    ConstString helpOff;
    bool *ptr;
    std::function<bool(bool val, bool isLoading, String *msg)> setFn;
    std::function<bool()> getFn;
    uint32_t event;
    uint32_t eventOn;
    uint32_t eventOff;
    BoolData(const char *name, bool isConstant);
    BoolData(String *name);
    virtual void addMenuInfo(JsonObject *menuInfo);
    virtual void addHelpInfo(String *msg);
    virtual void addStatusInfo(String *msg);
    virtual bool load(const JsonVariant &val, bool isCheckOnly, String *msg);
    virtual void save(JsonBuffer *buf, JsonObject *params);
  };

  class BoolDataBuilder {
  private:
    BoolData *data;
  public:
    BoolDataBuilder(const char *name, bool isConstant) {
      data = new BoolData(name, isConstant);
    }
    BoolDataBuilder(String *name) {
      data = new BoolData(name);
    }
    BoolDataBuilder &cmd(const char *cmd, bool isConstant) { data->cmd.set(cmd, isConstant); return *this; }
    BoolDataBuilder &cmd(const String &cmd) { data->cmd.set(cmd); return *this; }
    BoolDataBuilder &isPersistent(bool p) { data->isPersistent = p; return *this; }
    BoolDataBuilder &help(const char *h, bool isConstant) { data->help.set(h, isConstant); return *this; }
    BoolDataBuilder &help(const String &h) { data->help.set(h); return *this; }
    BoolDataBuilder &includeInStatus(bool b) { data->includeInStatus = b; return *this; }

    BoolDataBuilder &cmdOn(const char *cmdOn, bool isConstant) { data->cmdOn.set(cmdOn, isConstant); return *this; }
    BoolDataBuilder &cmdOn(const String &cmdOn) { data->cmdOn.set(cmdOn); return *this; }
    BoolDataBuilder &cmdOff(const char *cmdOff, bool isConstant) { data->cmdOff.set(cmdOff, isConstant); return *this; }
    BoolDataBuilder &cmdOff(const String &cmdOff) { data->cmdOff.set(cmdOff); return *this; }
    BoolDataBuilder &helpOn(const char *h, bool isConstant) { data->helpOn.set(h, isConstant); return *this; }
    BoolDataBuilder &helpOn(const String &h) { data->helpOn.set(h); return *this; }
    BoolDataBuilder &helpOff(const char *h, bool isConstant) { data->helpOff.set(h, isConstant); return *this; }
    BoolDataBuilder &helpOff(const String &h) { data->helpOff.set(h); return *this; }

    BoolDataBuilder &ptr(bool *ptr) { data->ptr = ptr; return *this; }
    BoolDataBuilder &setFn(std::function<bool(bool val, bool isLoading, String *msg)> fn) { data->setFn = fn; return *this; }
    BoolDataBuilder &getFn(std::function<bool()> fn) { data->getFn = fn; return *this; }

    BoolData *getData() const { return data; }
  };

  struct StringData: public ParamData {
    String *ptr;
    std::function<bool(const String &val, bool isLoading, String *msg)> setFn;
    std::function<void(String *val)> getFn;
    uint32_t event;
    StringData(const char *name, bool isConstant);
    StringData(const String *name);
    virtual void addStatusInfo(String *msg);
    virtual void addMenuInfo(JsonObject *menuInfo);
    virtual bool load(const JsonVariant &val, bool isCheckOnly, String *msg);
    virtual void save(JsonBuffer *buf, JsonObject *params);
  };

  class StringDataBuilder {
  private:
    StringData *data;
  public:
    StringDataBuilder(const char *name, bool isConstant) {
      data = new StringData(name, isConstant);
    }
    StringDataBuilder(const String *name) {
      data = new StringData(name);
    }
    StringDataBuilder &cmd(const char *cmd, bool isConstant) { data->cmd.set(cmd, isConstant); return *this; }
    StringDataBuilder &cmd(const String &cmd) { data->cmd.set(cmd); return *this; }
    StringDataBuilder &isPersistent(bool p) { data->isPersistent = p; return *this; }
    StringDataBuilder &help(const char *h, bool isConstant) { data->help.set(h, isConstant); return *this; }
    StringDataBuilder &help(const String &h) { data->help.set(h); return *this; }
    StringDataBuilder &includeInStatus(bool b) { data->includeInStatus = b; return *this; }

    StringDataBuilder &ptr(String *ptr) { data->ptr = ptr; return *this; }
    StringDataBuilder &setFn(std::function<bool(const String &val, bool isLoading, String *msg)> fn) { data->setFn = fn; return *this; }
    StringDataBuilder &getFn(std::function<void(String *)> fn) { data->getFn = fn; return *this; }

    StringData *getData() const { return data; }
  };

  struct SysPinData: public ParamData {
    SysPin *ptr;
    std::function<bool(int val, bool isLoading, String *msg)> setFn;
    std::function<int()> getFn;
    uint32_t event;
    SysPinData(const char *name, bool isConstant);
    SysPinData(const String *name);
    virtual void addStatusInfo(String *msg);
    virtual void addMenuInfo(JsonObject *menuInfo);
    virtual bool load(const JsonVariant &val, bool isCheckOnly, String *msg);
    virtual void save(JsonBuffer *buf, JsonObject *params);
  };

  class SysPinDataBuilder {
  private:
    SysPinData *data;
  public:
    SysPinDataBuilder(SysPin *pin) {
      data = new SysPinData(pin->getName(), true);
      data->cmd.set(pin->getName(), true);
      data->ptr = pin;
    }
    SysPinDataBuilder &isPersistent(bool p) { data->isPersistent = p; return *this; }
    SysPinDataBuilder &includeInStatus(bool p) { data->includeInStatus = p; return *this; }
    SysPinDataBuilder &help(const char *h, bool isConstant) { data->help.set(h, isConstant); return *this; }
    SysPinDataBuilder &help(const String h) { data->help.set(h); return *this; }
    SysPinDataBuilder &setFn(std::function<bool(int val, bool isLoading, String *msg)> fn) { data->setFn = fn; return *this; }

    SysPinData *getData() const { return data; }
  };

  struct JsonData: public ParamData {
    std::function<bool(const JsonVariant &val, bool isLoading, String *msg)> setFn;
    std::function<JsonVariant (JsonBuffer &buf)> getFn;
    uint32_t event;
    JsonData(const char *name, bool isConstant);
    JsonData(const String *name);
    virtual void addStatusInfo(String *msg);
    virtual void addMenuInfo(JsonObject *menuInfo);
    virtual bool load(const JsonVariant &val, bool isCheckOnly, String *msg);
    virtual void save(JsonBuffer *buf, JsonObject *params);
  };

  class JsonDataBuilder {
  private:
    JsonData *data;
  public:
    JsonDataBuilder(const char *name, bool isConstant) {
      data = new JsonData(name, isConstant);
    }
    JsonDataBuilder(const String *name) {
      data = new JsonData(name);
    }
    JsonDataBuilder &cmd(const char *cmd, bool isConstant) { data->cmd.set(cmd, isConstant); return *this; }
    JsonDataBuilder &cmd(const String &cmd) { data->cmd.set(cmd); return *this; }
    JsonDataBuilder &isPersistent(bool p) { data->isPersistent = p; return *this; }
    JsonDataBuilder &help(const char *h, bool isConstant) { data->help.set(h, isConstant); return *this; }
    JsonDataBuilder &help(const String &h) { data->help.set(h); return *this; }
    JsonDataBuilder &includeInStatus(bool b) { data->includeInStatus = b; return *this; }

    JsonDataBuilder &setFn(std::function<bool(const JsonVariant &val, bool isLoading, String *msg)> fn) { data->setFn = fn; return *this; }
    JsonDataBuilder &getFn(std::function<JsonVariant (JsonBuffer &buf)> fn) { data->getFn = fn; return *this; }

    JsonData *getData() const { return data; }
  };

  CommandMgr *cmdMgr;
  /** Used for the file name for loading and saving */
  String serviceName;
  int eventClass;
  std::vector<ParamData *> commandEntries;
  /**
   * Currently loaded config, empty if nothing loaded yet, by default
   * load and save to key "default".
   */
  String currentKeyName;

  std::function<bool(String *msg)> beforeLoadFn;
  std::function<void(String *msg)> afterLoadFn;
  std::function<bool(String *msg)> beforeSaveFn;
  std::function<void(String *msg)> afterSaveFn;
  std::function<bool(String *msg)> beforeStatusFn;
  std::function<void(String *msg)> afterStatusFn;

private:
  /** public: use CommandMgr::getServiceCommands() */
  ServiceCommands(const char *serviceName, CommandMgr *cmdMgr);
public:

  const char *getServiceName();

  void registerIntData(IntDataBuilder &data);
  void registerFloatData(FloatDataBuilder &data);
  void registerBoolData(BoolDataBuilder &data);
  void registerStringData(StringDataBuilder &data);
  void registerSysPinData(SysPinDataBuilder &data);
  void registerJsonData(JsonDataBuilder &data);

  void onBeforeLoad(std::function<bool(String *msg)> beforeLoadFn) { this->beforeLoadFn = beforeLoadFn; }
  void onAfterLoad(std::function<void(String *msg)> afterLoadFn) { this->afterLoadFn = afterLoadFn; }
  void onBeforeSave(std::function<bool(String *msg)> beforeSaveFn) { this->beforeSaveFn = beforeSaveFn; }
  void onAfterSave(std::function<void(String *msg)> afterSaveFn) { this->afterSaveFn = afterSaveFn; }

  /**
   * If returns true, the standard status message is not created, *msg is used. Else
   * the standard status message is appended to *msg.
   */
  void onBeforeStatus(std::function<bool(String *msg)> beforeStatusFn);
  /** Can manipulate output before it is shown, e.g., can add to *msg. */
  void onAfterStatus(std::function<void(String *msg)> afterStatusFn);

private:
  bool readConfigFile(DynamicJsonBuffer &buf, JsonObject **params, String *msg);
public:
  void getCurrentKeyName(String *keyName);
  bool listKeyNames(String *msg);
  bool save(const char *keyName, String *msg);
  /** keyName can be null to load the default config */
  bool load(const char *keyName, String *msg);

};


template <int maxSize>
class StringSplitter {
private:
    String *cmd;
    char buf[maxSize];
    int bufIdx;
    int i1;
    int i2;
public:
    StringSplitter(String *cmd) : cmd(cmd) {
        i1 = 0;
        i2 = 0;
        bufIdx = 0;
    }
    // If "prefix" is not null, it is added as a prefix to the word that's returned
    const char *nextWord(const char *prefix = nullptr) {
        while (i1 < cmd->length() && isspace(cmd->charAt(i1))) {
            ++i1;
        }
        i2 = i1;
        char *result = &buf[bufIdx];
        if (prefix != nullptr) {
            for (const char *p = prefix; *p != '\0' && bufIdx < (sizeof(buf) / sizeof(buf[0]) + 1); p++) {
                buf[bufIdx] = *p;
                ++bufIdx;
            }
        }
        while (i2 < cmd->length() && !isspace(cmd->charAt(i2)) && bufIdx < (sizeof(buf) / sizeof(buf[0]) + 1)) {
            buf[bufIdx] = cmd->charAt(i2);
            ++i2;
            ++bufIdx;
        }
        if (bufIdx < (sizeof(buf) / sizeof(buf[0]) + 1)) {
            buf[bufIdx] = '\0';
            ++bufIdx;
        }
        i1 = i2;
        return result;
    }

    const char *rest() {
        while (i1 < cmd->length() && isspace(cmd->charAt(i1))) {
            ++i1;
        }
        i2 = i1;
        char *result = &buf[bufIdx];
        while (i2 < cmd->length() && bufIdx < (sizeof(buf) / sizeof(buf[0]) + 1)) {
            buf[bufIdx] = cmd->charAt(i2);
            ++i2;
            ++bufIdx;
        }
        // go backwards removing any trailing spaces
        while (isspace(cmd->charAt(i2)) && i2 >= i1) {
            --i2;
            --bufIdx;
        }
        if (bufIdx < (sizeof(buf) / sizeof(buf[0]) + 1)) {
            buf[bufIdx] = '\0';
            ++bufIdx;
        }
        i1 = i2;
        return result;
    }
};

class CommandMgr {
private:
    UEventLoop *eventLoop;
    Monitor mon;
    SemaphoreHandle_t eventSem;

    int cmdInternalMenuInfo;
    int cmdInternalMenuList;

    std::vector<ServiceCommands*> serviceCommands;

  void getMenuList(String &buf);
  void getMenuInfo(const char *menuName, String &buf);

public:
    static bool getIntValue(UEvent *event, int *val);
    static bool getLongValue(UEvent *event, long *val);
    static bool getFloatValue(UEvent *event, float *val);
    static bool getDoubleValue(UEvent *event, double *val);
    static bool getBoolValue(UEvent *event, bool *val);
    static bool getStringValue(UEvent *event, String *val);

    CommandMgr();
    ~CommandMgr();

    void init(UEventLoop *eventLoop);

    UEventLoop *getEventLoop();

    void addCommand(const char *commandClass, const char *command);

    /**
     * Given a command line, generates an event
     *
     * Command line format:
     *     <class> <command> <arguments...>
     * Returns true if processed, processing result replaces the value in cmd
     **/
    bool processCommandLine(const char *channel, String *cmd);

    ServiceCommands *getServiceCommands(const char *serviceName);

};

#endif
