// #include <string.h>
// #include <time.h>
// #include <sys/time.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"
// #include "esp_system.h"
// #include "esp_event.h"
// #include "esp_log.h"
// #include "esp_attr.h"
// #include "esp_sleep.h"
// #include "nvs_flash.h"

// #include <lwip/apps/sntp.h>

// static const char *TAG = "example";

// /* Variable holding number of times ESP32 restarted since first boot.
//  * It is placed into RTC memory using RTC_DATA_ATTR and
//  * maintains its value when ESP32 wakes from deep sleep.
//  */
// RTC_DATA_ATTR static int boot_count = 0;

// static void obtain_time(void);
// static void initialize_sntp(void);

// #ifdef CONFIG_SNTP_TIME_SYNC_METHOD_CUSTOM
// void sntp_sync_time(struct timeval *tv)
// {
//    settimeofday(tv, NULL);
//    ESP_LOGI(TAG, "Time is synchronized from custom code");
//    sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
// }
// #endif

// void time_sync_notification_cb(struct timeval *tv)
// {
//     ESP_LOGI(TAG, "Notification of a time synchronization event");
// }

// void app_main()
// {
//     ++boot_count;
//     ESP_LOGI(TAG, "Boot count: %d", boot_count);

//     time_t now;
//     struct tm timeinfo;
//     time(&now);
//     localtime_r(&now, &timeinfo);
//     // Is time set? If not, tm_year will be (1970 - 1900).
//     if (timeinfo.tm_year < (2016 - 1900)) {
//         ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
//         obtain_time();
//         // update 'now' variable with current time
//         time(&now);
//     }
// #ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
//     else {
//         // add 500 ms error to the current system time.
//         // Only to demonstrate a work of adjusting method!
//         {
//             ESP_LOGI(TAG, "Add a error for test adjtime");
//             struct timeval tv_now;
//             gettimeofday(&tv_now, NULL);
//             int64_t cpu_time = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
//             int64_t error_time = cpu_time + 500 * 1000L;
//             struct timeval tv_error = { .tv_sec = error_time / 1000000L, .tv_usec = error_time % 1000000L };
//             settimeofday(&tv_error, NULL);
//         }

//         ESP_LOGI(TAG, "Time was set, now just adjusting it. Use SMOOTH SYNC method.");
//         obtain_time();
//         // update 'now' variable with current time
//         time(&now);
//     }
// #endif

//     char strftime_buf[64];

//     // Set timezone to Eastern Standard Time and print local time
//     setenv("TZ", "EST5EDT,M3.2.0/2,M11.1.0", 1);
//     tzset();
//     localtime_r(&now, &timeinfo);
//     strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
//     ESP_LOGI(TAG, "The current date/time in New York is: %s", strftime_buf);

//     // Set timezone to China Standard Time
//     setenv("TZ", "CST-8", 1);
//     tzset();
//     localtime_r(&now, &timeinfo);
//     strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
//     ESP_LOGI(TAG, "The current date/time in Shanghai is: %s", strftime_buf);

//     if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
//         struct timeval outdelta;
//         while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
//             adjtime(NULL, &outdelta);
//             ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
//                         outdelta.tv_sec,
//                         outdelta.tv_usec/1000,
//                         outdelta.tv_usec%1000);
//             vTaskDelay(2000 / portTICK_PERIOD_MS);
//         }
//     }

//     const int deep_sleep_sec = 10;
//     ESP_LOGI(TAG, "Entering deep sleep for %d seconds", deep_sleep_sec);
//     esp_deep_sleep(1000000LL * deep_sleep_sec);
// }

// static void obtain_time(void)
// {
//     ESP_ERROR_CHECK( nvs_flash_init() );
//     tcpip_adapter_init();
//     ESP_ERROR_CHECK( esp_event_loop_create_default() );

//     /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
//      * Read "Establishing Wi-Fi or Ethernet Connection" section in
//      * examples/protocols/README.md for more information about this function.
//      */
//     ESP_ERROR_CHECK(example_connect());

//     initialize_sntp();

//     // wait for time to be set
//     time_t now = 0;
//     struct tm timeinfo = { 0 };
//     int retry = 0;
//     const int retry_count = 10;
//     while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
//         ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
//     time(&now);
//     localtime_r(&now, &timeinfo);

//     ESP_ERROR_CHECK( example_disconnect() );
// }

// static void initialize_sntp(void)
// {
//     ESP_LOGI(TAG, "Initializing SNTP");
//     sntp_setoperatingmode(SNTP_OPMODE_POLL);
//     sntp_setservername(0, "pool.ntp.org");
//     sntp_set_time_sync_notification_cb(time_sync_notification_cb);
// #ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
//     sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
// #endif
//     sntp_init();
// }




// #include <time.h>

// const char* ntpServer = "de.pool.ntp.org";
// const long  gmtOffset_sec = 3600;
// const int   daylightOffset_sec = 3600;
// int second;
// int minute;
// int hour;
// int day;
// int month;
// int year;
// int weekday;
// long current;
// struct tm timeinfo;
//   /*
// struct tm
// {
// int    tm_sec;   //   Seconds [0,60]. 
// int    tm_min;   //   Minutes [0,59]. 
// int    tm_hour;  //   Hour [0,23]. 
// int    tm_mday;  //   Day of month [1,31]. 
// int    tm_mon;   //   Month of year [0,11]. 
// int    tm_year;  //   Years since 1900. 
// int    tm_wday;  //   Day of week [0,6] (Sunday =0). 
// int    tm_yday;  //   Day of year [0,365]. 
// int    tm_isdst; //   Daylight Savings flag. 
// }
//  */  

// void printLocalTime()
// {

//   if(!getLocalTime(&timeinfo)){
//     Serial.println("Failed to obtain time");
//     return;
//   }
//   Serial.println(&timeinfo, "%A, %d %B %Y %H:%M:%S");
// }

// void setup()
// {
//   Serial.begin(115200);
  
//   //connect to WiFi
//   Serial.printf("Connecting to %s ", ssid);
//   WiFi.begin(ssid, password);
//   while (WiFi.status() != WL_CONNECTED) {
//       delay(500);
//       Serial.print(".");
//   }
//   Serial.println(" CONNECTED");
  
//   //init and get the time
//   configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
//   printLocalTime();

//   //disconnect WiFi as it's no longer needed
//   WiFi.disconnect(true);
//   WiFi.mode(WIFI_OFF);
// }

// void loop()
// {
//   delay(1000);
  
//   printLocalTime();
//   second = timeinfo.tm_sec;
//   minute = timeinfo.tm_min;
//   hour = timeinfo.tm_hour;
//   day = timeinfo.tm_mday;
//   month = timeinfo.tm_mon + 1;
//   year = timeinfo.tm_year + 1900;
//   weekday = timeinfo.tm_wday +1;
//   Serial.print("Time from variables:  ");
//   Serial.print(day);
//   Serial.print(".");
//   Serial.print(month);
//   Serial.print(".");
//   Serial.print(year);
//   Serial.print(" --- ");
//   Serial.print(hour);
//   Serial.print(":");
//   Serial.print(minute);
//   Serial.print(":");
//   Serial.println(second);
// }