#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_system.h>
#include <nvs_flash.h>
#include <nvs.h>
#include <sdrone_controller_task.h>

static sdrone_state_t sdrone_state;
static sdrone_state_handle_t sdrone_state_handle = &sdrone_state;
static TaskHandle_t controller_task_handle;
static TaskHandle_t motors_task_handle;
static TaskHandle_t rc_task_handle;
static TaskHandle_t imu_task_handle;


esp_err_t my_rc_init_flash(){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
	return err;
}

void app_main(void)
{

	ESP_ERROR_CHECK(my_rc_init_flash());

	// init controller (this must be the first)
    xTaskCreate(sdrone_controller_task, "sdrone_controller_task", 4096, sdrone_state_handle, 5, &controller_task_handle);

    // init actuators
    xTaskCreate(sdrone_motors_task, "sdrone_motors_task", 4096, &(sdrone_state_handle->motors_state), 5, &motors_task_handle);

    // init sensors
    xTaskCreate(sdrone_rc_task, "sdrone_rc_task", 2048, &(sdrone_state_handle->rc_state), 10, &rc_task_handle);
    xTaskCreate(sdrone_imu_task, "sdrone_imu_task", 4096, &(sdrone_state_handle->imu_state), 10, &imu_task_handle);

    vTaskDelay(pdMS_TO_TICKS(100));

    sdrone_state_handle->motors_state.controller_task_handle = controller_task_handle;
    sdrone_state_handle->motors_state.driver_id = (uint32_t)SDRONE_MOTORS_DRIVER_ID;
    sdrone_state_handle->motors_state.controller_driver_id = (uint32_t)SDRONE_CONTROLLER_DRIVER_ID;

    sdrone_state_handle->rc_state.controller_task_handle = controller_task_handle;
    sdrone_state_handle->rc_state.driver_id = (uint32_t)SDRONE_RC_DRIVER_ID;
    sdrone_state_handle->rc_state.controller_driver_id = (uint32_t)SDRONE_CONTROLLER_DRIVER_ID;

    sdrone_state_handle->imu_state.controller_task_handle = controller_task_handle;
    sdrone_state_handle->imu_state.driver_id = (uint32_t)SDRONE_IMU_DRIVER_ID;
    sdrone_state_handle->imu_state.controller_driver_id = (uint32_t)SDRONE_CONTROLLER_DRIVER_ID;
}

