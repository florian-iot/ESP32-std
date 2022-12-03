#include <CompilationOpts.h>

#ifdef USE_LED_INDICATOR

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include <driver/ledc.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LedIndicatorService.h"
#include "LogMgr.h"

/*************************
 * 
 * 888                   888 8888888b.                              
 * 888                   888 888   Y88b                             
 * 888                   888 888    888                             
 * 888      .d88b.   .d88888 888   d88P 888  888  888 88888b.d88b.  
 * 888     d8P  Y8b d88" 888 8888888P"  888  888  888 888 "888 "88b 
 * 888     88888888 888  888 888        888  888  888 888  888  888 
 * 888     Y8b.     Y88b 888 888        Y88b 888 d88P 888  888  888 
 * 88888888 "Y8888   "Y88888 888         "Y8888888P"  888  888  888 
 *                                                                  
 *                                                                  
 *                                                                  
 * 
 *************************/



bool LedPwm::init(int pin0, int pin1, int pin2, bool isInverted,
    ledc_timer_t timer,
    ledc_channel_t channel0,
    ledc_channel_t channel1,
    ledc_channel_t channel2)
{
    for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
        animHead[j].animDef = nullptr;
        animHead[j].animHeadIdx = -1;
        animHead[j].state.lastStep = -1;
        animHead[j].state.nextStepTm = 0;
    }

    this->isInverted = isInverted;
    unsigned long tm = millis();
    for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
        lastFadeEndTm[j] = tm;
    }

    ledc_timer_config_t ledcTimer;
    ledcTimer.duty_resolution = LEDC_TIMER_13_BIT;
    ledcTimer.freq_hz = 5000;
    ledcTimer.speed_mode = LEDC_HIGH_SPEED_MODE;
    ledcTimer.timer_num = timer;

    esp_err_t rc = ledc_timer_config(&ledcTimer);
    if (rc != ESP_OK) {
        return false;
    }

    for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
        ledcChannel[j].channel = (j == 0 ? channel0 : j == 1 ? channel1 : channel2);
        ledcChannel[j].gpio_num = (gpio_num_t)(j == 0 ? pin0 : j == 1 ? pin1 : pin2);
        ledcChannel[j].speed_mode = LEDC_HIGH_SPEED_MODE;
        ledcChannel[j].timer_sel = timer;
        ledcChannel[j].intr_type = LEDC_INTR_DISABLE;
        ledcChannel[j].duty = (isInverted ? 8192 : 0);
        if (ledcChannel[j].gpio_num > 0) {
            rc = ledc_channel_config(&ledcChannel[j]);
            if (rc != ESP_OK) {
                return false;
            }
        }
    }

    // Initialize fade service.
    ledc_fade_func_install(0);

    return true;
}

void LedPwm::setDuty(int led, uint32_t duty, unsigned fadeMillis)
{
    if (ledcChannel[led].gpio_num == -1) {
        return;
    }
    unsigned long tm = millis();
    bool stopBeforeSetting = (tm - lastFadeEndTm[led] > 0);

    if (fadeMillis > 0) {
        if (stopBeforeSetting) {
            uint32_t oldDuty = ledc_get_duty(ledcChannel[led].speed_mode, ledcChannel[led].channel);
            ledc_stop(ledcChannel[led].speed_mode, ledcChannel[led].channel, isInverted ? HIGH : LOW);
            ledc_set_duty(ledcChannel[led].speed_mode, ledcChannel[led].channel, oldDuty);
            ledc_update_duty(ledcChannel[led].speed_mode, ledcChannel[led].channel);
        }
        ledc_set_fade_with_time(ledcChannel[led].speed_mode, ledcChannel[led].channel,
            isInverted ? 8192 - duty : duty, fadeMillis);
        ledc_fade_start(ledcChannel[led].speed_mode, ledcChannel[led].channel, LEDC_FADE_NO_WAIT);
        lastFadeEndTm[led] = tm + fadeMillis;
// Serial.printf("c%d fade to %d\n", led, duty);
    } else {
        if (stopBeforeSetting) {
            ledc_stop(ledcChannel[led].speed_mode, ledcChannel[led].channel, isInverted ? HIGH : LOW);
        }
        ledc_set_duty(ledcChannel[led].speed_mode, ledcChannel[led].channel, isInverted ? 8192 - duty : duty);
        ledc_update_duty(ledcChannel[led].speed_mode, ledcChannel[led].channel);
        lastFadeEndTm[led] = tm;
// Serial.printf("c%d set to %d\n", led, duty);
    }
}

int LedPwm::defineAnimation(const char *animation, String *error)
{
    // E.g.: c0:255,d500,0,d500;c1:0,f1000,255
    const char *p = animation;
    int n;
    int rc;
    int value;
    int durationType;
    int duration;

    AnimationDef anim;
    anim.channels[0] = nullptr;
    anim.channels[1] = nullptr;
    anim.channels[2] = nullptr;
    SingleChannelAnimationDef *curChannelAnimDef = nullptr;
    bool isOk = true;
    std::vector<AnimationDefEntry> curEntries;

    enum {
        CHANNEL,
        FIRST_VALUE,
        DURATION,
        VALUE,
        END
    };

    int state = CHANNEL;
    while (true) {
        while (*p != '\0' && isspace(*p)) {
            ++p;
        }
        switch (state) {
            case CHANNEL: {
                int channel;
                n = -1;
                rc = sscanf(p, "c%d:%n", &channel, &n);
                if (rc != 1 || n == -1) {
                    isOk = false;
                    *error = String("At char ") + (p - animation) + ": Expecting c<channel>:";
                    goto parseTerminated;
                }
                p += n;
                if (channel < 0 || channel > 2) {
                    isOk = false;
                    *error = String("At char ") + (p - animation) + ": Channel must be 0, 1 or 2";
                    goto parseTerminated;
                }
                if (anim.channels[channel] != nullptr) {
                    isOk = false;
                    *error = String("At char ") + (p - animation) + ": Chanel was already specified";
                    goto parseTerminated;
                }
                anim.channels[channel] = new SingleChannelAnimationDef();
                curChannelAnimDef = anim.channels[channel];
                curChannelAnimDef->channel = channel;
                curChannelAnimDef->startingDuty = -1;
                curChannelAnimDef->totalDuration = 0;
                state = FIRST_VALUE;
                break;
            }
            case VALUE:
            case FIRST_VALUE:
                n = -1;
                rc = sscanf(p, "%d%n", &value, &n);
                if (rc != 1 || n == -1) {
                    isOk = false;
                    *error = String("At char ") + (p - animation) + ": Expecting an integer value";
                    goto parseTerminated;
                }
                p += n;
                while (*p != '\0' && isspace(*p)) {
                    ++p;
                }

                if (state == FIRST_VALUE) {
                    curChannelAnimDef->startingDuty = value;
                } else {
                    curEntries[curEntries.size() - 1].duty = value;
                }

                if (*p == ',') {
                    ++p;
                    state = DURATION;
                } else if (*p == ';') {
                    ++p;
                    curChannelAnimDef->entries = new AnimationDefEntry[curEntries.size()];
                    for (int i = 0; i < curEntries.size(); i++) {
                        curChannelAnimDef->entries[i] = curEntries[i];
                    }
                    curChannelAnimDef->entriesCount = curEntries.size();
                    curEntries.clear();
                    state = CHANNEL;
                } else if (*p == '\0') {
                    curChannelAnimDef->entries = new AnimationDefEntry[curEntries.size()];
                    for (int i = 0; i < curEntries.size(); i++) {
                        curChannelAnimDef->entries[i] = curEntries[i];
                    }
                    curChannelAnimDef->entriesCount = curEntries.size();
                    curEntries.clear();
                    state = END;
                } else {
                    isOk = false;
                    *error = String("At char ") + (p - animation) + ": Expecting ',' or ';' or end of string after value";
                    goto parseTerminated;
                }

                break;
            case DURATION:
                if (*p == 'f' || *p == 'd') {
                    durationType = (*p == 'f' ? AnimationDefEntry::FADE : AnimationDefEntry::SET);
                    ++p;
                    n = -1;
                    rc = sscanf(p, "%d%n", &duration, &n);
                    if (rc != 1 || n == -1) {
                        isOk = false;
                        *error = String("At char ") + (p - animation) + ": Expecting duration after 'f' or 'd'";
                        goto parseTerminated;
                    }
                    p += n;
                    while (*p != '\0' && isspace(*p)) {
                        ++p;
                    }

                    curEntries.push_back(AnimationDefEntry());
                    curEntries[curEntries.size() - 1].type = durationType;
                    curEntries[curEntries.size() - 1].duration = duration;
                    curChannelAnimDef->totalDuration += duration;

                    if (*p == ',') {
                        ++p;
                        state = VALUE;
                    } else {
                        isOk = false;
                        *error = String("At char ") + (p - animation) + ": Expecting ',' after f<duration> or d<duration>";
                        goto parseTerminated;
                    }

                } else {
                    isOk = false;
                    *error = String("At char ") + (p - animation) + ": Expecting f<duration> or d<duration>";
                    goto parseTerminated;
                }
                break;
            case END:
                goto parseTerminated;
                break;
            default:
                isOk = false;
                *error = String("At char ") + (p - animation) + ": Invalid internal state";
                goto parseTerminated;
        }
    }
    parseTerminated:

    if (!isOk) {
        delete anim.channels[0]; anim.channels[0] = nullptr;
        delete anim.channels[1]; anim.channels[1] = nullptr;
        delete anim.channels[2]; anim.channels[2] = nullptr;
        return -1;
    }

    int animId = animationDefs.add(anim);
    for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
        if (anim.channels[j] != nullptr) {
            anim.channels[j]->animId = animId;
        }
    }
    return animId;
}

bool LedPwm::getAnimationDef(int id, String *str)
{
    // attn: can't keep the reference to anim, as animationDefs can be modified
    AnimationDef *anim = animationDefs.find(id);
    if (anim == nullptr) {
        return false;
    }
    auto printChannel = [this, str](SingleChannelAnimationDef *channel) {
        str->concat("c");
        str->concat(channel->channel);
        str->concat(":");
        str->concat(channel->startingDuty);
        for (int i = 0; i < channel->entriesCount; i++) {
            str->concat(",");
            AnimationDefEntry *e = &channel->entries[i];
            switch (e->type) {
                case AnimationDefEntry::SET: str->concat("d"); break;
                case AnimationDefEntry::FADE: str->concat("f"); break;
                default: str->concat("?"); break;
            }
            str->concat(e->duration);
            str->concat(",");
            str->concat(e->duty);
        }
    };
    int isFirst = true;
    for (int i = 0; i < LED_PWM_LEDCOUNT; i++) {
        if (anim->channels[i] != nullptr) {
            if (!isFirst) {
                str->concat(";");
            }
            printChannel(anim->channels[i]);
            isFirst = false;
        }
    }
    return true;
}

bool LedPwm::setAnimation(int level, int animId, uint32_t cycles)
{
    if (level < 0) {
        return false;
    }
    long tm = (long)millis();
    AnimationDef *anim = animationDefs.find(animId);
    if (anim == nullptr) {
        return false;
    }

    while (level >= animationStack.size()) {
        AnimationElement a;
        a.animId = -1;
        a.cycles = 0;
        for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
            a.channels[j] = nullptr;
            a.state[j].startTm = 0;
        }
        animationStack.push_back(a);
    }
    animationStack[level].animId = animId;
    animationStack[level].cycles = cycles;
    for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
        animationStack[level].channels[j] = anim->channels[j];
        animationStack[level].state[j].finishedCycles = 0;
        animationStack[level].state[j].startTm = tm;
    }

    recalcHead();

    return true;
}

void LedPwm::recalcHead()
{
    long tm = (long)millis();

    // calculate the current animations by going top-down in stack
    AnimHead newHead[LED_PWM_LEDCOUNT];
    for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
        newHead[j].animHeadIdx = -1;
        newHead[j].animDef = nullptr;
    }
    int found = 0; // count channels that we've already found, quit when 3
    for (int i = animationStack.size() - 1; found < LED_PWM_LEDCOUNT && i >= 0; i--) {
        for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
            if (animationStack[i].channels[j] != nullptr
                    && newHead[j].animHeadIdx == -1
                    && (animationStack[i].cycles == 0 || animationStack[i].state->finishedCycles < animationStack[i].cycles)) {
                newHead[j].animDef = animationStack[i].channels[j];
                newHead[j].animHeadIdx = i;
                newHead[j].state.lastStep = -1;
                newHead[j].state.nextStepTm = tm;
                ++found;
            }
        }
    }

    for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
        // if we're cleaning a channel, set its duty to 0
        if (newHead[j].animHeadIdx == -1 && animHead[j].animHeadIdx != -1) {
            setDuty(j, 0);
        }
    }

    for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
        if (animHead[j].animDef != newHead[j].animDef || animHead[j].animHeadIdx != newHead[j].animHeadIdx) {
            animHead[j] = newHead[j];
        }
    }

}

bool LedPwm::clearAnimation(int level, int animId)
{
    if (level >= animationStack.size() || level < 0) {
        return false;
    }
    if (animId != -1 && animationStack[level].animId != animId) {
        return false;
    }
    // clear this level, recalculate curAnimations for the channel.
    animationStack[level].animId = -1;
    animationStack[level].cycles = 0;
    for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
        animationStack[level].channels[j] = nullptr;
        animationStack[level].state[j].finishedCycles = 0;
        animationStack[level].state[j].startTm = 0;
    }
    recalcHead();
    return true;
}

uint32_t LedPwm::update()
{
    long tm = (long)millis();
    for (int i = 0; i < 3; i++) {
        if ((unsigned long)tm - lastFadeEndTm[i] > 7200000) { // if older than 2h, move it at 1h, so we can always compare with current millis()
            lastFadeEndTm[i] += 3600000;
        }
    }

// E.g.: c0:255,d500,0,f500,255;c1:0,f1000,255
    // see if we need to execute a change of pwm in current animation
    bool needRecalcHead = false;
    for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
        if (animHead[j].animHeadIdx == -1) {
            continue;
        }
        AnimationElement *animEl = &animationStack[animHead[j].animHeadIdx];
        SingleChannelAnimationDef *animDef = animEl->channels[j];

        bool didSet = false;
        if (animHead[j].state.nextStepTm - tm <= 0) { // we must execute a step item for this channel
            int curStep = animHead[j].state.lastStep;
            if (curStep == -1) {
                if (animEl->state->finishedCycles == 0) {
                    setDuty(j, animDef->startingDuty);
                    didSet = true;
                }
            } else {
                if (animDef->entries[curStep].type == AnimationDefEntry::SET) {
                    setDuty(j, animDef->entries[curStep].duty);
                    didSet = true;
                }
            }
            ++curStep;
            if (curStep >= animDef->entriesCount) {
                curStep = 0;
                ++animEl->state->finishedCycles;
            }
            if (animEl->cycles > 0 && animEl->state->finishedCycles >= animEl->cycles) {
                // this channel finished its cycles
                needRecalcHead = true;
            } else {
                if (animDef->entries[curStep].type == AnimationDefEntry::FADE) {
                    if (didSet) {
                        delay(1); // otherwise the set duty cycle won't be taken in account by fade
                    }
                    setDuty(j, animDef->entries[curStep].duty, animDef->entries[curStep].duration);                    
                }
            }

            animHead[j].state.lastStep = curStep;
            animHead[j].state.nextStepTm += (long)animDef->entries[curStep].duration;
        }
    }
    if (needRecalcHead) {
        recalcHead();
    }

    // update start times of the animation stack
    for (int i = 0; i < animationStack.size(); i++) {
        if (animationStack[i].animId != -1) {
            for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
                if (animHead[j].animHeadIdx == i) {
                    continue; // we've processed this as animHead
                }
                if (animationStack[i].channels[j] == nullptr) {
                    continue;
                }
                int totalDuration = animationStack[i].channels[j]->totalDuration;
                while ((animationStack[i].state[j].startTm + totalDuration) - tm < 0) {
                    animationStack[i].state[j].startTm += totalDuration;
                    ++animationStack[i].state[j].finishedCycles;
                }
            }
        }
    }

    // calculate next update
    long tillNextDuration = LONG_MAX; // will contain minimum duration to next step, all channels
    for (int j = 0; j < LED_PWM_LEDCOUNT; j++) {
        if (animHead[j].animHeadIdx == -1) {
            continue;
        }
        AnimationElement *animEl = &animationStack[animHead[j].animHeadIdx];
        if (animEl->cycles > 0 && animEl->state->finishedCycles >= animEl->cycles) {
            continue;
        }
        if (animHead[j].state.nextStepTm - tm < tillNextDuration) {
            tillNextDuration = animHead[j].state.nextStepTm - tm;
        }
    }

    return tillNextDuration > 0 ? (uint32_t)tillNextDuration : 0;
}

/*******************************************************************

 .d8888b.                            d8b                  
d88P  Y88b                           Y8P                  
Y88b.                                                     
 "Y888b.    .d88b.  888d888 888  888 888  .d8888b .d88b.  
    "Y88b. d8P  Y8b 888P"   888  888 888 d88P"   d8P  Y8b 
      "888 88888888 888     Y88  88P 888 888     88888888 
Y88b  d88P Y8b.     888      Y8bd8P  888 Y88b.   Y8b.     
 "Y8888P"   "Y8888  888       Y88P   888  "Y8888P "Y8888  

*****************************************************************/


void LedIndicatorService::init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr)
{
    this->eventLoop = eventLoop;
    this->commandMgr = commandMgr;
    this->logger = logMgr->newLogger("ledIndicator");

    ServiceCommands *cmd = commandMgr->getServiceCommands("ledIndicator");
    eventLoop->registerTimer(&timer);
    initCommands(cmd);

    // defaults
    pin0 = -1;
    pin1 = -1;
    pin2 = -1;
    isInverted = false;
    animationLevel = 0;

    // load config
    String msg;
    bool rc;
    rc = cmd->load(nullptr, &msg);
    if (rc) {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    // init data structures

    // init hardware
    ledPwm.init(pin0, pin1, pin2, isInverted);

    // timer
    isUpdating = false;

    timer.setCallback([this](UEventLoopTimer *timer) {
        uint32_t duration = ledPwm.update();
        if (duration == LONG_MAX) {
            isUpdating = false;
        } else {
            long d = duration == 0 ? 1 : duration;
            timer->setTimeout(d);
        }
    });
}

int LedIndicatorService::defineAnimation(const char *animation, String *error)
{
    return ledPwm.defineAnimation(animation, error);
}

bool LedIndicatorService::getAnimationDef(int id, String *str)
{
    return ledPwm.getAnimationDef(id, str);
}

bool LedIndicatorService::setAnimation(int level, int animId, uint32_t cycles)
{
    bool rc = ledPwm.setAnimation(level, animId, cycles);
    if (!rc) {
        return false;
    }
    timer.cancelTimeout();
    long duration = ledPwm.update();
    if (duration == LONG_MAX) {
        isUpdating = false;
    } else {
        isUpdating = true;
        timer.setTimeout(duration);
    }
    return true;
}

bool LedIndicatorService::clearAnimation(int level, int animId)
{
    bool rc = ledPwm.clearAnimation(level, animId);
    if (!rc) {
        return false;
    }

    timer.cancelTimeout();
    long duration = ledPwm.update();
    if (duration == LONG_MAX) {
        isUpdating = false;
    } else {
        isUpdating = true;
        timer.setTimeout(duration);
    }
    return true;
}


void LedIndicatorService::initCommands(ServiceCommands *cmd)
{
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("pin0", true)
        .cmd("pin0")
        .help("--> Set led pin 0, set to -1 for no led; requires reboot.")
        .vMin(-1)
        .vMax(99)
        .ptr(&pin0)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            pin0 = val;
            *msg = "Led pin set to "; *msg += pin0; *msg += ", please reboot.";
            return true;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("pin1", true)
        .cmd("pin1")
        .help("--> Set led pin 1, set to -1 for no led; requires reboot.")
        .vMin(-1)
        .vMax(99)
        .ptr(&pin1)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            pin1 = val;
            *msg = "Led pin set to "; *msg += pin1; *msg += ", please reboot.";
            return true;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("pin2", true)
        .cmd("pin2")
        .help("--> Set led pin 2, set to -1 for no led; requires reboot.")
        .vMin(-1)
        .vMax(99)
        .ptr(&pin2)
        .setFn([this](int val, bool isLoading, String *msg) -> bool {
            pin2 = val;
            *msg = "Led pin set to "; *msg += pin2; *msg += ", please reboot";
            return true;
        })
    );
    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("inverted", true)
        .cmd("inverted")
        .help("inverted on | off --> If set, the duty represents the low part of the signal, not the high one.")
        .ptr(&isInverted)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("fade", true)
        .cmd("fade")
        .help("--> Test fade")
        .isPersistent(false)
        .setFn([this](int val, bool isLoading, String *msg) {
            ledPwm.setDuty(0, val, 5000);
            *msg = "Fading";
            return true;
        })
    );
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("set", true)
        .cmd("set")
        .help("--> Test set")
        .isPersistent(false)
        .setFn([this](int val, bool isLoading, String *msg) {
            ledPwm.setDuty(0, val, 0);
            *msg = "Setting";
            return true;
        })
    );

    cmd->registerStringData(
        ServiceCommands::StringDataBuilder("defineAnimation", true)
        .cmd("defineAnimation")
        .help("--> define an animation, syntax example: c0:0,f250,255,d500,255,f250,0;c1:...")
        .isPersistent(false)
        .setFn([this](const String &val, bool isLoading, String *msg) {
            String error;
            int testAnimationId = defineAnimation(val.c_str(), &error);
            if (testAnimationId == -1) {
                *msg = "Error: ";
                msg->concat(error);
            } else {
                String testAnimation = val;
                *msg = "Animation defined: id "; msg->concat(testAnimationId);
                msg->concat(" (");
                getAnimationDef(testAnimationId, msg);
                msg->concat(")");
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("showAnimation", true)
        .cmd("showAnimation")
        .help("--> display the given animation ")
        .isPersistent(false)
        .setFn([this](int val, bool isLoading, String *msg) {
            msg->clear();
            bool rc = getAnimationDef(val, msg);
            if (!rc) {
                *msg = "Unable to display animation with id ";
                *msg += val;
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("animationLevel", true)
        .cmd("animationLevel")
        .help("--> Set level for a future set animation")
        .isPersistent(false)
        .vMin(0)
        .vMax(5)
        .ptr(&animationLevel)
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("setAnimation", true)
        .cmd("setAnimation")
        .help("--> Set given animation, until changed")
        .isPersistent(false)
        .setFn([this](int val, bool isLoading, String *msg) {
            bool rc = setAnimation(animationLevel, val, 0);
            if (rc) {
                *msg = "Animation "; msg->concat(val); msg->concat(" set at level ");
                msg->concat(animationLevel);
                msg->concat(", cycles: no limit");
            } else {
                *msg = "Unable to set animation "; msg->concat(val); msg->concat(" at level ");
                msg->concat(animationLevel);
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("setAnimation3", true)
        .cmd("setAnimation3")
        .help("--> Set given animation, for 3 cycles")
        .isPersistent(false)
        .setFn([this](int val, bool isLoading, String *msg) {
            bool rc = setAnimation(animationLevel, val, 3);
            if (rc) {
                *msg = "Animation "; msg->concat(val); msg->concat(" set at level ");
                msg->concat(animationLevel);
                msg->concat(", for 3 cycles");
            } else {
                *msg = "Unable to set animation "; msg->concat(val); msg->concat(" at level ");
                msg->concat(animationLevel);
            }
            return true;
        })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("setAnimation10", true)
        .cmd("setAnimation10")
        .help("--> Set given animation, for 10 cycles")
        .isPersistent(false)
        .setFn([this](int val, bool isLoading, String *msg) {
            bool rc = setAnimation(animationLevel, val, 10);
            if (rc) {
                *msg = "Animation "; msg->concat(val); msg->concat(" set at level ");
                msg->concat(animationLevel);
                msg->concat(", for 10 cycles");
            } else {
                *msg = "Unable to set animation "; msg->concat(val); msg->concat(" at level ");
                msg->concat(animationLevel);
            }
            return true;
        })
    );

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("clearAnimation", true)
        .cmdOn("clearAnimation")
        .helpOn("--> Clear the animation at the level set by \"animationLevel\"")
        .isPersistent(false)
        .setFn([this](bool val, bool isLoading, String *msg) {
            bool rc = clearAnimation(animationLevel);
            if (rc) {
                *msg = "Animation cleared from level ";
                msg->concat(animationLevel);
            } else {
                *msg = "Unable to clear animation from level ";
                msg->concat(animationLevel);
            }
            return true;
        })
    );
}

#endif
