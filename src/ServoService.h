#include <CompilationOpts.h>

#ifdef USE_SERVO
#ifndef INCL_SERVO_H
#define INCL_SERVO_H

#include <HardwareSerial.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "UEvent.h"
#include "CommandMgr.h"
#include "LogMgr.h"
#include <ESP32Servo.h>
#include <TFT_eSPI.h>

class ServoService {
public:
    void init(UEventLoop *eventLoop, CommandMgr *commandMgr, LogMgr *logMgr);
private:
    UEventLoop *eventLoop;
    CommandMgr *commandMgr;
    ServiceCommands *cmd;
    Logger *logger;

    Servo servo;
    UEventLoopTimer drivingTimer;
    UEventLoopTimer positioningTimer;

    int servoPin;
    int autoIdleMillis;
    int minPulseWidth;
    int maxPulseWidth;
    int position;
    int maxPosition;
    int slack; // slack for positioning, if >0, then positioning CCW is done by going first to pos - slack, then to pos.
    int slackPositioningDurationMillis; // milliseconds to wait for positioning to slack position before going to final position
    bool isPrePositioning;

    int lcdBacklightLevel;
    ESP32PWM lcdBacklightPwm;

    bool lcdAnimate;
    int lcdLoopCnt;
    int lcdFps;
    bool lcdIsDma;
    int lcdBgColor;

    bool isEnabled;

    void initCommands();
    void enable();
    void disable();
    void setLcdBacklight(int val, bool isInit, String *msg);

    TFT_eSPI tft;
    TFT_eSprite *spr[2];
    // Toggle buffer selection
    bool sprSel;
    // Pointers to start of Sprties in RAM
    uint16_t* sprPtr[2];
    uint16_t palette[6];
    // Used for fps measuring
    uint16_t counter;
    long startMillis;
    uint16_t interval;
    // size / 2 of cube edge
    float d;
    float px[8], py[8], pz[8];
    int faces[12][3];
    float p2x[8], p2y[8];
    float r[3];
    // Sprite draw position
    int16_t xpos;
    int16_t ypos;
    // 3 axis spin control
    bool spinX;
    bool spinY;
    bool spinZ;
    // Min and max of cube edges, "int" type used for compatibility with original sketch min() function
    int xmin, ymin, xmax, ymax;

    uint32_t updateTime;       // time for next update
    bool bounce;
    int wait; //random (20);

    // Random movement direction
    int dx;
    int dy;


    UEventLoopTimer  lcdTimer;


    void lcdInit();
    void lcdLoop();

    int shoelace(int x1, int y1, int x2, int y2, int x3, int y3);
    void drawCube();


};

#endif
#endif