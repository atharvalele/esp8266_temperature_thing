/*
 * app_main: project entry point
 *
 * Sets up all tasks to do
 */

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bmp280.h"
#include "ntp.h"
#include "wifi_conn.h"

void app_main()
{
    wifi_init_sta();
    
    struct bmp280_device bmp280 = {
        .i2c_addr = BMP280_I2C_ADDR_PRIM,
        .mode = BMP280_FORCE,
        .temp_os = BMP280_OS_1X,
        .pres_os = BMP280_OS_1X,
    };
    
    xTaskCreate(
        bmp280_task,
        "Sensor Task",
        1024,
        &bmp280,
        1,
        NULL
    );

    xTaskCreate(
        ntp_task,
        "NTP Task",
        2048,
        NULL,
        3,
        NULL
    );

    /*
     * Suspend the main task so that our bmp280 struct
     * is still in memory.. need to find a better way
     * to handle this
     */
    vTaskSuspend(NULL);
}
