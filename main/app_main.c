/*
 * app_main: project entry point
 *
 * Sets up all tasks to do
 */

#include <stdio.h>

#include "bmp280.h"

void app_main()
{
    struct bmp280_device bmp280;
    
    bmp280_init(&bmp280);
}
