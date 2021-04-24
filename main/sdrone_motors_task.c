/*
 * sdrone_motors_task.c
 *
 *  Created on: 19 apr 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <motors.h>

void sdrone_motors_task_init(motors_handle_t motors_handle) {
	printf("sdrone_motors_task_init::Started\n");
	memset(motors_handle, 0, sizeof(*motors_handle));

	// Init motor left
	motors_handle->motor[0].enabled = true;
	motors_handle->motor[0].pin = GPIO_NUM_21;
	motors_handle->motor[0].position = LEFT;

	// Init motor right
	motors_handle->motor[1].enabled = true;
	motors_handle->motor[1].pin = GPIO_NUM_22;
	motors_handle->motor[1].position = RIGHT;

	printf("sdrone_motors_task_init::Init Motors\n");
	ESP_ERROR_CHECK(motors_init(motors_handle));
}

void sdrone_motors_task(void *arg) {
	motors_t motors;
	motors_handle_t motors_handle = &motors;
	sdrone_motors_task_init(motors_handle);
    vTaskDelay(100); //delay of 1s

	printf("sdrone_motors_task_init::Arm motors without program esc\n");
	ESP_ERROR_CHECK(motors_arm(motors_handle));

    vTaskDelay(10); //delay of 1/10s
    float duty_min = 40.0f;
    float duty_max = 99.0f;
    float increment = 1.0f;
	while(true) {
		printf("Cycle motors\n");
		//TODO: aggiornare il duty
		if(motors_handle->motor[0].duty_cycle >= duty_max) {
			increment = -1.0f;
		}
		if(motors_handle->motor[0].duty_cycle <= duty_min) {
			increment = 1.0f;
		}
		motors_handle->motor[0].duty_cycle += increment;
		motors_handle->motor[1].duty_cycle += increment;
		motors_update(motors_handle);
	    vTaskDelay(100);
	};

}
