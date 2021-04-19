#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <sdrone_motors_task.h>

void app_main(void)
{
    xTaskCreate(sdrone_motors_task, "sdrone_motors_task", 4096, NULL, 5, NULL);
}

