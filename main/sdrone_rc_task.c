/*
 * sdrone_rc_task.c
 *
 *  Created on: 17 mag 2021
 *      Author: andrea
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <rc.h>
#include <sdrone_rc_task.h>

void sdrone_rc_task(void *arg) {
	sdrone_rc_state_handle_t sdrone_rc_state_handle =
			(sdrone_rc_state_handle_t) arg;
	rc_handle_t rc_handle = &(sdrone_rc_state_handle->rc_data);
	rc_t rc_data_local;
	rc_t* rc_data_local_handle = &rc_data_local;

	ESP_ERROR_CHECK(rc_init(rc_data_local_handle));
	sdrone_rc_state_handle->rc_task_handle = xTaskGetCurrentTaskHandle();

	while (true) {
		ESP_ERROR_CHECK(rc_start(rc_data_local_handle));
		if (rc_data_local_handle->data.txrx_signal == RC_TXRX_TRANSMITTED) {
			if ((sdrone_rc_state_handle->controller_task_handle != NULL) && (rc_handle->data.txrx_signal != RC_TXRX_TRANSMITTED)) {
				memcpy(rc_handle, rc_data_local_handle,
						sizeof(*rc_data_local_handle));
				xTaskNotify(sdrone_rc_state_handle->controller_task_handle,
						sdrone_rc_state_handle->driver_id,
						eSetValueWithOverwrite);
			}
		}
	}
	vTaskDelete(NULL);

}
