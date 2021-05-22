/*
 * sdrone_controller_task.c
 *
 *  Created on: 18 mag 2021
 *      Author: andrea
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <sdrone_controller_task.h>

void sdrone_controller_init(sdrone_state_handle_t sdrone_state_handle) {
	printf("sdrone_controller_init init initial state and motors\n");
	memset(sdrone_state_handle, 0, sizeof(*sdrone_state_handle));
	sdrone_state_handle->driver_id = (uint32_t) SDRONE_CONTROLLER_DRIVER_ID;
	printf("sdrone_controller_init initial state sdrone initialized\n");
}

void sdrone_controller_cycle(sdrone_state_handle_t sdrone_state_handle) {
	// TODO: Implement cycle
	// TODO: Ricevere notifiche dai sensori (imu e RC). Ho inserito in sdrone_state_t il riferimento al task del controller
	// TODO: se tramite RC si richiede arm/disarm dei motori settarlo in sdrone_motors_input_t.desired_motors_status
	// TODO: Modificare sdrone_motors_task in modo che prenda in considerazione i comandi (arm/disarm) ricevuti in input
	//       Eventualmente prevedere una struct Motors_Control e creare una union con sdrone_motors_input_t

	uint32_t rc_channels[RC_MAX_CHANNELS];
	mpu9250_data_t imu_data;

	uint8_t counter = 0;
	while (true) {
		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) != 0) {
			if(sdrone_state_handle->rc_state.rc_data.data.txrx_signal == RC_TXRX_TRANSMITTED) {
				memcpy(rc_channels, sdrone_state_handle->rc_state.rc_data.data.channels, sizeof(*rc_channels)*RC_MAX_CHANNELS);
				sdrone_state_handle->rc_state.rc_data.data.txrx_signal = RC_TXRX_RECEIVED;
			}
			if(sdrone_state_handle->imu_state.imu.data.txrx_signal == IMU_TXRX_TRANSMITTED) {
				counter++;
				counter %= 100;
				memcpy(&imu_data, &sdrone_state_handle->imu_state.imu.data, sizeof(imu_data));
				sdrone_state_handle->imu_state.imu.data.txrx_signal = IMU_TXRX_RECEIVED;
				if(counter == 0) {
					printf("Gyro.: S[%d][%d][%d] X[%d][%d][%d]\n", imu_data.gyro.cal.kalman[X_POS].sample, imu_data.gyro.cal.kalman[Y_POS].sample, imu_data.gyro.cal.kalman[Z_POS].sample, imu_data.gyro.cal.kalman[X_POS].X , imu_data.gyro.cal.kalman[Y_POS].X, imu_data.gyro.cal.kalman[Z_POS].X);
					printf("Accel: S[%d][%d][%d] X[%d][%d][%d]\n", imu_data.accel.cal.kalman[X_POS].sample, imu_data.accel.cal.kalman[Y_POS].sample, imu_data.accel.cal.kalman[Z_POS].sample, imu_data.accel.cal.kalman[X_POS].X , imu_data.accel.cal.kalman[Y_POS].X, imu_data.accel.cal.kalman[Z_POS].X);
					printf("AG[%2.2f][%2.2f][%2.2f] RPY[%2.2f][%2.2f][%2.2f]\n", imu_data.attitude[X_POS], imu_data.attitude[Y_POS], imu_data.attitude[Z_POS], imu_data.gyro.rpy.xyz.x*(double)360.0f/(double)6.283185307f, imu_data.gyro.rpy.xyz.y*(double)360.0f/(double)6.283185307f, imu_data.gyro.rpy.xyz.z*(double)360.0f/(double)6.283185307f);
					printf("AA[%2.2f][%2.2f][%2.2f] RPY[%2.2f][%2.2f][%2.2f]\n", imu_data.attitude[X_POS], imu_data.attitude[Y_POS], imu_data.attitude[Z_POS], imu_data.accel.rpy.xyz.x*(double)360.0f/(double)6.283185307f, imu_data.accel.rpy.xyz.y*(double)360.0f/(double)6.283185307f, imu_data.accel.rpy.xyz.z*(double)360.0f/(double)6.283185307f);
					printf("KG[%1.5f][%1.5f][%1.5f]\n", imu_data.gyro.cal.kalman[X_POS].K, imu_data.gyro.cal.kalman[Y_POS].K, imu_data.gyro.cal.kalman[Z_POS].K );
				}
			}
//			for (uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
//				printf("%d-%04d ", i,
//						rc_channels[i]);
//			}
//			printf("\n");
//			sdrone_state_handle->motors_state.input.isCommand = true;
//			sdrone_state_handle->motors_state.input.command.type = MOTORS_ARM;
//			if (sdrone_state_handle->motors_state.motors_task_handle != NULL) {
//				xTaskNotify(
//						sdrone_state_handle->motors_state.motors_task_handle,
//						sdrone_state_handle->driver_id, eSetValueWithOverwrite);
//			}
		} else {
			printf("NOT PASS\n");
		}
	}
}
void sdrone_controller_task(void *arg) {
	sdrone_state_handle_t sdrone_state_handle = (sdrone_state_handle_t) arg;
	sdrone_controller_init(sdrone_state_handle);
	sdrone_controller_cycle(sdrone_state_handle);
	vTaskDelete(NULL);
}
