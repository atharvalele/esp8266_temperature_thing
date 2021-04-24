/*
 * NTP for Timesyncing
 *
 * Code taken from ESP8266 RTOS SDKs SNTP Example
 * Ref: https://github.com/espressif/ESP8266_RTOS_SDK/blob/master/examples/protocols/sntp
 */

#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"

#include "nvs.h"
#include "nvs_flash.h"

#include "lwip/apps/sntp.h"

#define MAX_RETRY_COUNT 10

/* Global Time Container */
struct tm system_time;

static void init_sntp()
{
    ESP_LOGI("NTP", "Initialzing NTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();

    /*
     * Set timezone
     * IST is 5h30m ahead of UTC.
     * Ref: https://www.gnu.org/software/libc/manual/html_node/TZ-Variable.html
     */
    setenv("TZ", "UTC-5:30", 1);
    tzset();
}

void ntp_task(void *args)
{
    time_t now;
    char timebuf[64];

    int retry = 0;
    const int retry_count = 10;

    ESP_ERROR_CHECK(esp_netif_init());

    init_sntp();

    while ((system_time.tm_year < (2016 - 1900)) && (++retry < retry_count)) {
        ESP_LOGI("NTP", "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &system_time);
    }

    while (1) {
        // update 'now' variable with current time
        time(&now);
        localtime_r(&now, &system_time);

        if (system_time.tm_year < (2016 - 1900)) {
            ESP_LOGE("NTP", "Datetime out of sync!");
        } else {
            strftime(timebuf, sizeof(timebuf), "%c", &system_time);
            ESP_LOGI("NTP", "Current Time: %s", timebuf);
        }

        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}