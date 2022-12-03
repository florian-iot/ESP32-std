#include "CompilationOpts.h"

#ifdef USE_LED

#include <HardwareSerial.h>
#include <FS.h>
#include <SPIFFS.h>
#include <FastLED.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LedService.h"
#include "LedSparkEffect.h"


EffectSparks::EffectSparks(int controllerId, LedMap1d *leds, int totalLedCount, int defaultLedStart, int defaultLedCount, const char *description)
    : Effect(controllerId, leds, totalLedCount, description)
{
    attackValPerFrame = nullptr;
    decayValPerFrame = nullptr;
    ledState = nullptr;
    isEnabled = false;

    ledState = nullptr;
    attackValPerFrame = nullptr;
    decayValPerFrame = nullptr;

    this->ledStart = defaultLedStart;
    this->ledCount = defaultLedCount;
}

EffectSparks::~EffectSparks()
{
    delete[] attackValPerFrame;
    delete[] decayValPerFrame;
    delete[] ledState;
}

void EffectSparks::setCmd(ServiceCommands *cmd, const char *command, const char *help, uint8_t *data, int min, int max)
{
  cmd->registerIntData(
    ServiceCommands::IntDataBuilder(command, true)
    .cmd(command, true)
    .help(help)
    .vMin(min)
    .vMax(max)
    .setFn([this, data, command](int val, bool isLoading, String *msg) {
      *data = val;
      if (!isLoading) {
        calcFrameProperties();
        *msg = "Set "; msg->concat(command);
        msg->concat(" to "); msg->concat(*data);
      }
      return true;
    })
    .getFn([this, data]() {
      return *data;
    })
  );
};

void EffectSparks::setCmd(ServiceCommands *cmd, const char *command, const char *help, int *data, int min, int max)
{
  cmd->registerIntData(
    ServiceCommands::IntDataBuilder(command, true)
    .cmd(command, true)
    .help(help)
    .vMin(min)
    .vMax(max)
    .setFn([this, data, command](int val, bool isLoading, String *msg) {
      *data = val;
      if (!isLoading) {
        calcFrameProperties();
        *msg = "Set "; msg->concat(command);
        msg->concat(" to "); msg->concat(*data);
      }
      return true;
    })
    .getFn([this, data]() {
      return *data;
    })
  );
};

void EffectSparks::init(int millisPerFrame, CommandMgr *commandMgr, uint32_t frame)
{
    this->millisPerFrame = millisPerFrame;

    String controllerName = "led"; controllerName += String(controllerId);
    cmd = commandMgr->getServiceCommands(controllerName.c_str());

    cmd->onAfterLoad([this](String *msg) { this->onAfterLoad(); });

    cmd->registerBoolData(
        ServiceCommands::BoolDataBuilder("isEnabled", true)
            .cmdOn("enable")
            .cmdOff("disable")
            .helpOn("--> Enable this controller")
            .helpOff("--> Disable this controller")
            .ptr(&isEnabled)
            .setFn([this](bool val, bool isLoading, String *msg) {
                if (isLoading) {
                    isEnabled = val;
                } else {
                    if (isEnabled != val) {
                        isEnabled = val;
                        *msg = "Controller "; msg->concat(cmd->getServiceName());
                        msg->concat(isEnabled ? " enabled" : " disabled");
                    } else {
                        *msg = "Controller "; msg->concat(cmd->getServiceName());
                        msg->concat(" is already");
                        msg->concat(isEnabled ? " enabled" : " disabled");
                    }
                }
                return true;
            })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("ledStart", true)
            .cmd("ledStart")
            .help("--> Starting led, in hardware order; sets the ledCount to 1")
            .vMin(0)
            .vMax(totalLedCount - 2)
            .setFn([this](int val, bool isLoading, String *msg) {
                ledStart = val;
                ledCount = 1;
                *msg = "Set ledStart to "; msg->concat(ledStart); msg->concat(", led count to 1");
                return true;
            })
            .getFn([this]() {
                return ledStart;
            })
    );

    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("ledCount", true)
            .cmd("ledCount")
            .help("--> Number of leds, consecutive to ledStart, in hardware order")
            .vMin(0)
            .setFn([this](int val, bool isLoading, String *msg) {
                if (ledStart + val > this->totalLedCount) {
                    *msg = "Led count "; msg->concat(val); msg->concat(" out of range, given ledStart = ");
                    msg->concat(ledStart);
                    msg->concat(" for totalLedCount = "); msg->concat(totalLedCount);
                    msg->concat(", maximum is "); msg->concat(totalLedCount - ledStart);
                } else {
                    ledCount = val;
                    *msg = "Set ledCount to "; msg->concat(ledCount);
                }
                return true;
            })
            .getFn([this]() {
                return ledCount;
            })
    );
    setCmd(cmd, "valMin", "--> Minimum brightness, 0..255", &valMin, 0, 255);
    setCmd(cmd, "valMax", "--> Maximum brightness, 0..255", &valMax, 0, 255);
    setCmd(cmd, "valIdle", "--> Brightness when not sparking, default 0 (led off), 0..255", &valIdle, 0, 255);
    setCmd(cmd, "hue1", "--> Hue 1, 0..255", &hueParams.hue1, 0, 255);
    setCmd(cmd, "hue2", "--> Hue 2, 0..255", &hueParams.hue2, 0, 255);
    setCmd(cmd, "saturation1", "--> Saturation 1, 0..255", &hueParams.sat1, 0, 255);
    setCmd(cmd, "saturation2", "--> Saturation 2, 0..255", &hueParams.sat2, 0, 255);
    setCmd(cmd, "hue1HighDelta", "--> Hue 1 delta when high, 0..255", &hueParams.hue1HighDelta, 0, 255);
    setCmd(cmd, "hue2HighDelta", "--> Hue 2 delta when high, 0..255", &hueParams.hue2HighDelta, 0, 255);
    setCmd(cmd, "hueJitter", "--> Random change in hue (uniform), 0..255", &hueParams.hueJitter, 0, 255);
    setCmd(cmd, "hue1Hue2Pct", "--> Percentage of mix between hue 1 and hue 2, 0..100 (100 means 100% hue1)", &hueParams.hue1Hue2Pct, 0, 100);
    setCmd(cmd, "hueXScale", "--> Hue change factor in X direction of leds, 256 gives a change of 1 for each led from preceding one",
        &hueXScale, 0, 65536);
    setCmd(cmd, "hueTScale", "--> Hue change factor in time, 265 gives a change of 1 for each frame", &hueTScale, 0, 65536);
    setCmd(cmd, "sparkDensityPct", "--> Spark density, 1..100", &sparkDensityPct, 0, 100);
    setCmd(cmd, "sparkDurationMillis", "--> Spark duration in millis", &sparkDurationMillis, 0, INT_MAX);
    setCmd(cmd, "sparkAttackTimePct", "--> Spark attack time, in percentage (1..100)", &sparkAttackTimePct, 0, 100);
    setCmd(cmd, "sparkDecayTimePct", "--> Spark decay time, in percentage (1..100)", &sparkDecayTimePct, 0, 100);
    cmd->registerIntData(
        ServiceCommands::IntDataBuilder("sparkGamma", true)
            .cmd("sparkGamma")
            .help("--> Gamma to use for spark attack and decay, *100 (100 == gamma 1.0)")
            .vMin(1)
            .vMax(INT_MAX)
            .setFn([this](int val, bool isLoading, String *msg) {
                sparkGamma = val / 100.0;
                if (!isLoading) {
                    calcFrameProperties();
                    *msg = "Set sparkGamma to ";
                    msg->concat(sparkGamma * 100);
                }
                return true;
            })
            .getFn([this]() {
                return sparkGamma * 100;
            })
    );

    isEnabled = true;

    ledStart = 0;
    ledCount = totalLedCount;

    String msg;
    bool rc = cmd->load(nullptr, &msg);
    if (!rc) {
        Serial.printf("LedController %s: error loading defaults: %s\n", cmd->getServiceName(), msg.c_str());
    } else {
        String keyName;
        cmd->getCurrentKeyName(&keyName);
        Serial.printf("Loaded config for %s/%s\n", cmd->getServiceName(), keyName.c_str());
    }

    this->millisPerFrame = millisPerFrame;
    attackValPerFrame = nullptr;
    decayValPerFrame = nullptr;
    expDistribMean = 0;
    calcFrameProperties();
    lastFrame = frame - 2; // so that we reinit led state, since frame != lastFrame + 1
    lastLedCount = ledCount;
    lastSparkFrames = sparkFrames;
    lastNonSparkFrames = nonSparkFrames;
    statAvgSparkingPct = 0;
}

void EffectSparks::onAfterLoad()
{
    calcFrameProperties();
}

void EffectSparks::getOneLineStatus(String *msg)
{
    *msg += "Type: spark, leds start at "; *msg += this->ledStart; *msg += ", for "; *msg += this->ledCount; *msg += " leds. Controller is ";
    *msg += (this->isEnabled ? "enabled." : "disabled.");
}


void EffectSparks::setFrameDuration(int millis)
{
    this->millisPerFrame = millis;
    calcFrameProperties();
}

void EffectSparks::calcFrameProperties()
{
    sparkFrames = sparkDurationMillis / millisPerFrame;
    if (sparkFrames == 0) {
        sparkFrames = 1;
    }
    attackFrames = sparkAttackTimePct * sparkFrames / 100;
    if (attackFrames == 0) {
        attackFrames = 1;
    }
    decayFrames = sparkDecayTimePct * sparkFrames / 100;
    if (decayFrames == 0) {
        decayFrames = 1;
    }
    fullValFrames = sparkFrames - attackFrames - decayFrames;
    if (fullValFrames <= 0) {
        fullValFrames = 1;
    }
    sparkFrames = attackFrames + fullValFrames + decayFrames;

    nonSparkFrames = sparkFrames * (100 - sparkDensityPct) / sparkDensityPct;
    if (nonSparkFrames == 0) {
        nonSparkFrames = 1;
    }
    // We'll want a random variable with exponential distribution
    // with mean nonSparkFrames.
    // Given uniform rnd X ]0..1], we get an exponential rnd
    // with the formula -ln(X)*nonSparkFrames.
    // We construct an array with -ln(X/1024)*nonSparkFrames values for X in 1..1024
    // which can be used by simply indexing with X.
    // We clip output values between 1 and 10*nonSparkFrames.

    if (expDistribMean != nonSparkFrames) {
// Serial.printf("Exponential distrib: ");
        int slots = sizeof(expDistribTable) / sizeof(expDistribTable[0]);
        for (int i = 0; i < slots; i++) {
            double expVal = -log((i + 1) / (double)slots) * nonSparkFrames;
            if (expVal < 1) {
                expDistribTable[i] = 1;
            } else if (expVal >= 10 * nonSparkFrames) {
                expDistribTable[i] = 10 * nonSparkFrames;
            } else {
                expDistribTable[i] = (int)(expVal + 0.5);
            }
// Serial.printf("%d ", expDistribTable[i]);
        }
// Serial.printf("\n");
        expDistribMean = nonSparkFrames;
    }

    delete [] attackValPerFrame;
    attackValPerFrame = new uint8_t[attackFrames];
    delete [] decayValPerFrame;
    decayValPerFrame = new uint8_t[decayFrames];

    double valMinInvGamma = pow(valMin, 1.0 / sparkGamma);
    double valMaxInvGamma = pow(valMax, 1.0 / sparkGamma);

    if (attackFrames == 1) {
        attackValPerFrame[0] = valMax;
    } else {
        double error = 0.0; // cumulate rounding error, then add it to next frame
        for (int i = 0; i < attackFrames; i++) {
            uint64_t i1 = (uint64_t)i;
            uint64_t i2 = i1 * i1;
            uint64_t i3 = i2 * i1;
            uint64_t d1 = attackFrames - 1;
            uint64_t d2 = d1 * d1;
            uint64_t d3 = d2 * d1;
            double easing = 3.0 * (double)i2 / (double)d2 - 2.0 * (double)i3 / (double)d3;
            double val = pow(valMinInvGamma + easing * (valMaxInvGamma - valMinInvGamma), sparkGamma);

#if 0
TODO
            // handle dithering: 1. value for each frame must be different, 2. The error on the
            // value, compared to the calculated, should be the smallest possible.
            // For this, given a theoritical value, find the integer value V that is closest to
            // the theoritical value, check values V-1, V, V+1 excluding any that equals
            // the previous value, take the one that minimizes the error.

            int v = (uint8_t)(val + error + 0.5);
            int finalV;
            double err = 1e99; // we're pretty sure this is higher than any error we can calculat
            if (v - 1 != previousVal) {
                finalV = v - 1;
                err = val - finalV;
            }
            if (v != previousVal) {
                if (abs(val - v) < abs(err)) { // we've a better match
                    finalV = v;
                    err = val - finalV;
                }
            }
            if (v + 1 != previousVal) {
                if (abs(val - (v + 1)) < abs(err)) { // we've a better match
                    finalV = v + 1;
                    err = val - finalV;
                }
            }
            attackValPerFrame[i] = (uint8_t)finalV;
            error += err;
#endif
            attackValPerFrame[i] = (uint8_t)(val + error + 0.5);
            error += val - attackValPerFrame[i];
        }
    }
    if (decayFrames == 1) {
        decayValPerFrame[0] = valMin;
    } else {
        double error = 0.0; // cumulate rounding error, then add it to next frame
        for (int i = 0; i < decayFrames; i++) {
            uint64_t i1 = (uint64_t)i;
            uint64_t i2 = i1 * i1;
            uint64_t i3 = i2 * i1;
            uint64_t d1 = decayFrames - 1;
            uint64_t d2 = d1 * d1;
            uint64_t d3 = d2 * d1;
            double easing = 3.0 * (double)i2 / (double)d2 - 2.0 * (double)i3 / (double)d3;
            double val = pow(valMinInvGamma + easing * (valMaxInvGamma - valMinInvGamma), sparkGamma);

            decayValPerFrame[decayFrames - 1 - i] = (uint8_t)(val + error + 0.5);
            error += val - (double)decayValPerFrame[decayFrames - 1 - i];
        }
    }

Serial.printf("Properties: sparkFrames %d (%d/%d\\%d), nonSparkFrames %d\n",
    sparkFrames, attackFrames, fullValFrames, decayFrames, nonSparkFrames);
Serial.printf("   Attack: ");
for (int i = 0; i < attackFrames; i++) {
    Serial.printf(" %d ", attackValPerFrame[i]);
}
Serial.printf("\n   Decay: ");
for (int i = 0; i < decayFrames; i++) {
    Serial.printf(" %d ", decayValPerFrame[i]);
}
Serial.printf("\n");

}

int EffectSparks::getRndExp()
{
    const int sz = (sizeof(expDistribTable) / sizeof(expDistribTable[0]));
    int r = rnd.nextUInt() % (sz << 4);
    int slot = (r >> 4);
    int dec = (r & 0x0F);
    int result;
    if (slot < sz - 1) {
        result = (expDistribTable[slot] * dec + expDistribTable[slot + 1] * (16 - dec)) >> 4;
    } else {
        result = expDistribTable[slot];
    }
// Serial.printf("Got uniform random %d, exp random %d, at slot %d:%d\n", r, result, slot, dec);
    return result;
}

void EffectSparks::calc(uint32_t frame)
{
    if (!isEnabled) {
        return;
    }

    bool isInitState = (lastFrame != frame - 1);
    lastFrame = frame;
    if (ledCount != lastLedCount || ledState == nullptr) {
        if (ledState != nullptr) {
            delete[] ledState;
        }
        ledState = new LedState[ledCount];
        lastLedCount = ledCount;
        isInitState = true;
    }
    if (isInitState) {
        for (int i = 0; i < ledCount; i++) {
            int rndVal = (rnd.nextUInt() % sparkFrames) + getRndExp();
            ledState[i].lastOn = frame + rndVal;
            ledState[i].hueKind = rnd.bernoulli(hueParams.hue1Hue2Pct, 100) ? 0 : 1;
            ledState[i].baseHue = (ledState[i].hueKind == 0) ? hueParams.hue1 : hueParams.hue2;
        }
    }

    if (sparkFrames != lastSparkFrames || nonSparkFrames != lastNonSparkFrames) {
        // set new timings
        // reset timings for existing leds
        for (int i = 0; i < ledCount; i++) {
            if (ledState[i].lastOn > frame) {
                // leds that are not sparking
                ledState[i].lastOn = frame + (rnd.nextUInt() % (sparkFrames / 2)) + getRndExp();
            } else {
                // leds that are sparking, lastOn + sparkFrames must remain in the future
                // if not, make them non-sparking (they'll shut down immediately, without easing)
                if (ledState[i].lastOn + sparkFrames < frame) {
                    ledState[i].lastOn = frame + (rnd.nextUInt() % (sparkFrames / 2)) + getRndExp();
                }
            }
        }
        lastSparkFrames = sparkFrames;
        lastNonSparkFrames = nonSparkFrames;
    }

    // if we're at the last frame of sparking,
    // calculate next time we need to enter sparking
    int sparkingCount = 0; // for stats
    for (int i = 0; i < ledCount; i++) {
        if (frame == ledState[i].lastOn + sparkFrames) {
            ledState[i].lastOn = frame + getRndExp();
            // change hue
            ledState[i].hueKind = (rnd.nextUInt() % 100) <  hueParams.hue1Hue2Pct ? 0 : 1;
            ledState[i].baseHue =
                (ledState[i].hueKind == 0 ? hueParams.hue1 : hueParams.hue2)
                // hueTScale
                + frame * hueTScale / 256
                // hueXScale
                + i * hueXScale / 256;
            if (hueParams.hueJitter > 0) {
                uint8_t jitter = (rnd.nextUInt() % hueParams.hueJitter) - (hueParams.hueJitter / 2);
                ledState[i].baseHue += jitter;
            }
        }
        int lastOn = ledState[i].lastOn;
        CHSV color;
        if (frame < lastOn) {
            if (valIdle == 0) {
                color.v = 0;
            } else {
                color.setHSV(ledState[i].baseHue, ledState[i].hueKind == 0 ? hueParams.sat1 : hueParams.sat2, valIdle);
            }
        } else if (frame < lastOn + attackFrames) {
            int f = frame - lastOn;
            int val = attackValPerFrame[f];
            int hue =
                // starting hue
                ledState[i].baseHue
                // delta
                + (ledState[i].hueKind == 0 ? hueParams.hue1HighDelta : hueParams.hue2HighDelta) * f / attackFrames;
            color.setHSV(hue, ledState[i].hueKind == 0 ? hueParams.sat1 : hueParams.sat2, val);
            ++sparkingCount;
        } else if (frame < lastOn + attackFrames + fullValFrames) {
            int hue =
                // starting hue
                ledState[i].baseHue
                // delta
                + (ledState[i].hueKind == 0 ? hueParams.hue1HighDelta : hueParams.hue2HighDelta);
            color.setHSV(hue, ledState[i].hueKind == 0 ? hueParams.sat1 : hueParams.sat2, valMax);
            ++sparkingCount;
        } else if (frame < lastOn + attackFrames + fullValFrames + decayFrames) {
            int f = frame - lastOn - attackFrames - fullValFrames;
            int val = decayValPerFrame[f];
            int hue =
                // starting hue
                ledState[i].baseHue
                // delta
                + (ledState[i].hueKind == 0 ? hueParams.hue1HighDelta : hueParams.hue2HighDelta) * (decayFrames - f) / decayFrames;
            color.setHSV(hue, ledState[i].hueKind == 0 ? hueParams.sat1 : hueParams.sat2, val);
            ++sparkingCount;
        } else {
            if (valIdle == 0) {
                color.v = 0;
            } else {
                color.setHSV(ledState[i].baseHue, ledState[i].hueKind == 0 ? hueParams.sat1 : hueParams.sat2, valIdle);
            }
        }

        // last effect wins unless it is 0
        if (color != CRGB(0,0,0)) {
            *leds->at(ledStart + i) = color;
        }

//         // stongest luma wins
//         if (ledArray[ledStart + i] == CRGB(0,0,0)) {
//             hsv2rgb_rainbow(color, ledArray[i]);
//             // hsv2rgb_spectrum(color, ledArray[i]);
// //            ledArray[i].g /= 2;
//         } else { // strongest luma wins
//             int newLuma = color.v; // getLuma() ?
//             int currentLuma = ledArray[i].getLuma();
//             if (newLuma > currentLuma) {
//                 hsv2rgb_rainbow(color, ledArray[i]);
//                 // hsv2rgb_spectrum(color, ledArray[i]);
// //                ledArray[i].g /= 2;
//             }
//         }

    }
    if (statAvgSparkingPct == 0) {
        statAvgSparkingPct = ((uint64_t)sparkingCount * 100 / ledCount) << 16;
    } else {
        statAvgSparkingPct = ((uint64_t)statAvgSparkingPct * 255 + ((uint64_t)sparkingCount << 16) * 100 / ledCount) / 256;
    }
}

void EffectSparks::printStats()
{
    Serial.printf("  %s -- Avg sparking: %d%%\n", cmd->getServiceName(), (statAvgSparkingPct >> 16));
}

void EffectSparks::printStats(String *msg)
{
    *msg += cmd->getServiceName(); *msg += " -- Avg sparking: "; *msg += (statAvgSparkingPct >> 16); *msg += "%";
}



#endif // USE_LED
