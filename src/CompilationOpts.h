#ifndef INC_COMPILATION_OPTS_H
#define INC_COMPILATION_OPTS_H

// #define USE_MONITOR_TEST

#define USE_EVENT_CHECK_HEAP 1

// Logging
#define USE_LOGGING
#define LOGGING_ENABLE_TESTS


//#define USE_LM75A_TEMP
#define LM75A_TEMP_I2CADDRESS 0x48

// TODO time not implemented yet
#define USE_TIME
#define TIME_NTP_SERVER "fr.pool.ntp.org"
// for time zone see (https://remotemonitoringsystems.ca/time-zone-abbreviations.php)
#define TIME_TZ_INFO "CET-1CEST-2,M3.5.0/02:00:00,M10.5.0/03:00:00"
#define TIME_USE_DS3231 true

// #define USE_LED
#define LED_CONTROLLERS 16
#define LED_PIN_1 4
#define LED_PIN_2 16
#define LED_PIN_3 17
#define LED_PIN_4 18
#define LED_PIN_5 19
#define LED_PIN_6 23
#define LED_PIN_7 22
#define LED_PIN_8 25
#define LED_PIN_9 26
#define LED_PIN_10 27

#define USE_HEARTBEAT
#define HEARTBEAT_PIN 2

#define USE_SPIFFS_EDITOR
#define SPIFFS_EDITOR_USER "admin"
#define SPIFFS_EDITOR_PWD "admin"

#define USE_STEPPER

#define STEPPER_ENABLE_PIN 23
#define STEPPER_M0_PIN 22
#define STEPPER_M1_PIN 21
#define STEPPER_M2_PIN 19
#define STEPPER_RESET_PIN 18
#define STEPPER_SLEEP_PIN 17
#define STEPPER_STEP_PIN 16
#define STEPPER_DIR_PIN 4
#define STEPPER_FAULT_PIN 34
#define STEPPER_STOPPER_PIN 26
#define STEPPER_STOPPER_POWERSENSE_PIN 36
#define STEPPER_LED_PIN 32
#define STEPPER_BUZZER_PIN 33
#define STEPPER_DEBUG_PIN -1

// pins for which to compile suppport in FastLED
#define STEPPER_LED_PIN_1 32
#define STEPPER_LED_PIN_2 33
#define STEPPER_LED_PIN_3 25
#define STEPPER_LED_PIN_4 26
#define STEPPER_LED_PIN_5 27
#define STEPPER_LED_PIN_6 22
#define STEPPER_LED_PIN_7 23
#define STEPPER_LED_PIN_8 4
#define STEPPER_LED_PIN_9 16
#define STEPPER_LED_PIN_10 17


//#define USE_JS




#define AUTOCONNECT_PSK "31415926"
#define WIRE0_SDA 21
#define WIRE0_SCL 22

#endif
