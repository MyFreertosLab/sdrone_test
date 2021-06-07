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
#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
	esp_err_t sdrone_controller_horizontal_hexacopter_init(sdrone_state_handle_t sdrone_state_handle) {
	    // TODO: T.B.D.
		return ESP_OK;
	}
#else
#ifdef MOTORS_FRAME_TWO_HORIZONTAL_AXIS
	esp_err_t sdrone_controller_two_horizontal_axis_init(
			sdrone_state_handle_t sdrone_state_handle) {
		// TODO: T.B.D.
		return ESP_OK;
	}
#endif
#endif
}

#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
esp_err_t sdrone_controller_horizontal_hexacopter_control(sdrone_state_handle_t sdrone_state_handle) {
    // TODO: T.B.D.
	return ESP_OK;
}
#else
#ifdef MOTORS_FRAME_TWO_HORIZONTAL_AXIS
esp_err_t sdrone_controller_two_horizontal_axis_control(
		sdrone_state_handle_t sdrone_state_handle) {
	float X[3] = { 0.0f, 0.0f, 0.0f }; // the new state
	float Y[2] = { 0.0f, 0.0f }; // the new response

	// Update U from RC
	sdrone_state_handle->controller_state.U[SDRONE_TETA_POS] =
			sdrone_state_handle->rc_state.rc_data.data.norm[RC_ROLL]
					* SDRONE_NORM_ROLL_TO_RADIANS_FACTOR
					- sdrone_state_handle->controller_state.X[SDRONE_TETA_POS];
	sdrone_state_handle->controller_state.U[SDRONE_THRUST_POS] =
			sdrone_state_handle->rc_state.rc_data.data.norm[RC_THROTTLE]
					* SDRONE_NORM_THROTTLE_TO_ACCEL_FACTOR*((float)SDRONE_NUM_MOTORS);

	// Update X from IMU
	sdrone_state_handle->controller_state.X[SDRONE_TETA_POS] =
			sdrone_state_handle->imu_state.imu.data.gyro.rpy.xyz.x;
	float prevOmega = sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS];
	sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS] =
			(double) sdrone_state_handle->imu_state.imu.data.gyro.cal.kalman[X_POS].X
					/ (double) sdrone_state_handle->imu_state.imu.data.gyro.lsb
					/ (double) 360.0f * (double) PI_2;
	sdrone_state_handle->controller_state.X[SDRONE_ALFA_POS] =
			((double) sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS]
					- prevOmega);
//					* (double) sdrone_state_handle->imu_state.imu.data_rate;

	// new state (only for prediction)
	X[SDRONE_TETA_POS] =
			sdrone_state_handle->controller_state.X[SDRONE_TETA_POS]
					+ sdrone_state_handle->controller_state.U[SDRONE_TETA_POS];
	X[SDRONE_OMEGA_POS] =
			sdrone_state_handle->controller_state.U[SDRONE_TETA_POS]
					/ SDRONE_REF_SIGNAL_DT;
	X[SDRONE_ALFA_POS] =
			-sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS]
					/ SDRONE_REF_SIGNAL_DT
					+ sdrone_state_handle->controller_state.U[SDRONE_TETA_POS]
							/ SDRONE_REF_SIGNAL_DT;

	// response
	Y[0] = -sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS]*SDRONE_AXIS_LENGTH/(2.0f)
			+ sdrone_state_handle->controller_state.U[SDRONE_TETA_POS]
					* SDRONE_AXIS_LENGTH / (2.0f * SDRONE_REF_SIGNAL_DT)
			+ 0.5f * sdrone_state_handle->controller_state.U[SDRONE_THRUST_POS];
	Y[1] = sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS]*SDRONE_AXIS_LENGTH/(2.0f)
			- sdrone_state_handle->controller_state.U[SDRONE_TETA_POS]
					* SDRONE_AXIS_LENGTH / (2.0f * SDRONE_REF_SIGNAL_DT)
			+ 0.5f * sdrone_state_handle->controller_state.U[SDRONE_THRUST_POS];

	printf("XP[%5.3f, %5.3f, %5.3f]\n", X[0], X[1], X[2]);
	printf("XC[%5.3f, %5.3f, %5.3f]\n",
			sdrone_state_handle->controller_state.X[SDRONE_TETA_POS],
			sdrone_state_handle->controller_state.X[SDRONE_OMEGA_POS],
			sdrone_state_handle->controller_state.X[SDRONE_ALFA_POS]);
	printf("Y[%5.3f, %5.3f]\n", Y[0], Y[1]);
	printf("U[%5.3f, %5.3f]\n", sdrone_state_handle->controller_state.U[SDRONE_TETA_POS], sdrone_state_handle->controller_state.U[SDRONE_THRUST_POS]);
	return ESP_OK;
}
#endif
#endif

void sdrone_controller_cycle(sdrone_state_handle_t sdrone_state_handle) {
	rc_data_t rc_data;

	mpu9250_data_t imu_data;
	while (true) {
		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) == pdPASS) {
			memcpy(&rc_data, &sdrone_state_handle->rc_state.rc_data.data,
					sizeof(rc_data));
			memcpy(&imu_data, &sdrone_state_handle->imu_state.imu.data,
					sizeof(imu_data));
			break;
		} else {
			printf("wait for initialization complete ...\n");
		}
	}
	uint16_t counter = 0;
	while (true) {
		if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100)) != 0) {
			counter++;
			counter %= 200;
			if (sdrone_state_handle->rc_state.rc_data.data.txrx_signal
					== RC_TXRX_TRANSMITTED) {
				memcpy(&rc_data, &sdrone_state_handle->rc_state.rc_data.data,
						sizeof(rc_data));
				sdrone_state_handle->rc_state.rc_data.data.txrx_signal =
						RC_TXRX_RECEIVED;
			}
			if (sdrone_state_handle->imu_state.imu.data.txrx_signal
					== IMU_TXRX_TRANSMITTED) {
				memcpy(&imu_data, &sdrone_state_handle->imu_state.imu.data,
						sizeof(imu_data));
				sdrone_state_handle->imu_state.imu.data.txrx_signal =
						IMU_TXRX_RECEIVED;
				if (counter == 0) {
					//			printf("%2.3f, %d;\n",
					//					sdrone_state_handle->motors_state.input.data.thrust,
					//					sdrone_state_handle->imu_state.imu.data.accel.cal.kalman[Z_POS].X
					//				  );
//					printf("Gyro.: S[%d][%d][%d] X[%d][%d][%d]\n", imu_data.gyro.cal.kalman[X_POS].sample, imu_data.gyro.cal.kalman[Y_POS].sample, imu_data.gyro.cal.kalman[Z_POS].sample, imu_data.gyro.cal.kalman[X_POS].X , imu_data.gyro.cal.kalman[Y_POS].X, imu_data.gyro.cal.kalman[Z_POS].X);
//					printf("Accel: S[%d][%d][%d] X[%d][%d][%d]\n", imu_data.accel.cal.kalman[X_POS].sample, imu_data.accel.cal.kalman[Y_POS].sample, imu_data.accel.cal.kalman[Z_POS].sample, imu_data.accel.cal.kalman[X_POS].X , imu_data.accel.cal.kalman[Y_POS].X, imu_data.accel.cal.kalman[Z_POS].X);
//					printf("AG[%2.2f][%2.2f][%2.2f] RPY[%2.2f][%2.2f][%2.2f]\n", imu_data.attitude[X_POS], imu_data.attitude[Y_POS], imu_data.attitude[Z_POS], imu_data.gyro.rpy.xyz.x*(double)360.0f/(double)6.283185307f, imu_data.gyro.rpy.xyz.y*(double)360.0f/(double)6.283185307f, imu_data.gyro.rpy.xyz.z*(double)360.0f/(double)6.283185307f);
//					printf("AA[%2.2f][%2.2f][%2.2f] RPY[%2.2f][%2.2f][%2.2f]\n", imu_data.attitude[X_POS], imu_data.attitude[Y_POS], imu_data.attitude[Z_POS], imu_data.accel.rpy.xyz.x*(double)360.0f/(double)6.283185307f, imu_data.accel.rpy.xyz.y*(double)360.0f/(double)6.283185307f, imu_data.accel.rpy.xyz.z*(double)360.0f/(double)6.283185307f);
//					printf("KG[%1.5f][%1.5f][%1.5f]\n", imu_data.gyro.cal.kalman[X_POS].K, imu_data.gyro.cal.kalman[Y_POS].K, imu_data.gyro.cal.kalman[Z_POS].K );

#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
					ESP_ERROR_CHECK(sdrone_controller_horizontal_hexacopter_control(sdrone_state_handle));
#else
#ifdef MOTORS_FRAME_TWO_HORIZONTAL_AXIS
					ESP_ERROR_CHECK(
							sdrone_controller_two_horizontal_axis_control(
									sdrone_state_handle));
#endif
#endif
					sdrone_state_handle->motors_state.input.isCommand = false;
					sdrone_state_handle->motors_state.input.data.thrust =
							(rc_data.norm[RC_THROTTLE])
									* SDRONE_NORM_THROTTLE_TO_ACCEL_FACTOR;
					sdrone_state_handle->motors_state.input.data.tx_rx_flag =
							MOTORS_TXRX_TRANSMITTED;
					if (sdrone_state_handle->motors_state.motors_task_handle
							!= NULL) {
						xTaskNotify(
								sdrone_state_handle->motors_state.motors_task_handle,
								sdrone_state_handle->driver_id,
								eSetValueWithOverwrite);
					}
				}
			}
		} else {
			// printf("NOT PASS\n");
		}
	}
}

void sdrone_controller_task(void *arg) {
	sdrone_state_handle_t sdrone_state_handle = (sdrone_state_handle_t) arg;
	sdrone_controller_init(sdrone_state_handle);
	sdrone_controller_cycle(sdrone_state_handle);
	vTaskDelete(NULL);
}
