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
#include <sdrone_motors_task.h>
#include <driver/i2c.h>
#include <ina3221.h>

static void sdrone_motors_controller_init(
		sdrone_motors_state_handle_t sdrone_motors_state_handle) {
	printf("sdrone_motors_controller_init init initial state and motors\n");
	memset(sdrone_motors_state_handle, 0, sizeof(*sdrone_motors_state_handle));
	ESP_ERROR_CHECK(motors_init(&(sdrone_motors_state_handle->motors)));

	sdrone_motors_state_handle->motors_task_handle =
			xTaskGetCurrentTaskHandle();

	// voltage & current sensor (3 channels);
	printf("sdrone_motors_controller_init init ina3221 sensor\n");
	ina3221_t ina3221;
	ina3221_handle_t ina3221_handle = &ina3221;
	ina3221_handle->i2c_master_port = MOTORS_INA3221_I2C_PORT;
	ESP_ERROR_CHECK(ina3221_init(ina3221_handle));
	ESP_ERROR_CHECK(ina3221_test_connection(ina3221_handle));
	sdrone_motors_state_handle->ina3221_handle = ina3221_handle;
	printf("sdrone_motors_controller_init initial state and motors initialized\n");
}

esp_err_t sdrone_motors_two_horizontal_axis_control(
		sdrone_motors_state_handle_t sdrone_motors_state_handle) {
	return ESP_OK;
}
esp_err_t sdrone_motors_horizontal_hexacopter_control(
		sdrone_motors_state_handle_t sdrone_motors_state_handle) {
	return ESP_OK;
}
void sdrone_motors_controller_cycle(
		sdrone_motors_state_handle_t sdrone_motors_state_handle) {
	motors_handle_t motors_handle = &(sdrone_motors_state_handle->motors);

	printf("sdrone_motors_task_init::Arm motors\n");
	ESP_ERROR_CHECK(motors_arm(motors_handle));
	vTaskDelay(pdMS_TO_TICKS(10));

	/* TODO:
	 * 1) configurare il timer0 in modo da ricevere un interrupt prima che inizi la nuova fase
	 * 2) ricevere notifica da isr
	 * 3) ricevuta la notifica, calcolare il valore Y(i+1) ed eseguire update motors
	 *    - update motors deve avvenire prima dell'inizio della fase successiva.
	 *    - quindi il task deve avere priorità massima
	 */
	while (true) {
		// Take Notify at end of duty_cycle
		if (ulTaskNotifyTake( pdTRUE, pdMS_TO_TICKS(10))
				== sdrone_motors_state_handle->controller_driver_id) {
			if (sdrone_motors_state_handle->input.isCommand) {
				switch (sdrone_motors_state_handle->input.command.type) {
				case MOTORS_ARM: {
					ESP_ERROR_CHECK(motors_arm(motors_handle));
					break;
				}
				case MOTORS_DISARM: {
					ESP_ERROR_CHECK(motors_disarm(motors_handle));
					break;
				}
				case MOTORS_SWITCHOFF: {
					ESP_ERROR_CHECK(motors_switchoff(motors_handle));
					break;
				}
				case MOTORS_SWITCHON: {
					ESP_ERROR_CHECK(motors_switchon(motors_handle));
					break;
				}
				default: {
					printf(
							"ERROR: sdrone_motors_controller_cycle::command %d unknown for motors",
							sdrone_motors_state_handle->input.command.type);
				}
				}
			} else if (sdrone_motors_state_handle->input.data.tx_rx_flag
					== MOTORS_TXRX_TRANSMITTED) {
				/*
				 * load input data to local var
				 */
				float nd = 0;
				ESP_ERROR_CHECK(motors_newton_to_duty(sdrone_motors_state_handle->input.data.thrust, &motors_handle->motor[1].duty_cycle));
				ESP_ERROR_CHECK(motors_duty_to_newton(motors_handle->motor[1].duty_cycle, &nd));
				printf("MOTOR 1: N[%5.5f],D[%5.5f], ND[%5.5f]\n", sdrone_motors_state_handle->input.data.thrust, motors_handle->motor[1].duty_cycle, nd);
				ESP_ERROR_CHECK(motors_update(motors_handle));
				sdrone_motors_state_handle->input.data.tx_rx_flag =
						MOTORS_TXRX_RECEIVED;
			}
			/*
			 * calc Y(i) for each motor
			 * calc duty_cycle for each motor
			 * not interrompible
			 *   update motors
			 */

#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
	      ESP_ERROR_CHECK(motors_config_horizontal_hexacopter_control(sdrone_motors_state_handle));
#else
#ifdef MOTORS_FRAME_TWO_HORIZONTAL_AXIS
			ESP_ERROR_CHECK(
					sdrone_motors_two_horizontal_axis_control(
							sdrone_motors_state_handle));
#endif
#endif
		}
	}
}

void sdrone_motors_task(void *arg) {
	sdrone_motors_state_handle_t sdrone_motors_state_handle =
			(sdrone_motors_state_handle_t) arg;
	sdrone_motors_controller_init(sdrone_motors_state_handle);
	sdrone_motors_controller_cycle(sdrone_motors_state_handle);

}
