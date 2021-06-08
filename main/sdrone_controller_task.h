/*
 * sdrone_controller_task.h
 *
 *  Created on: 18 mag 2021
 *      Author: andrea
 */

#ifndef MAIN_SDRONE_CONTROLLER_TASK_H_
#define MAIN_SDRONE_CONTROLLER_TASK_H_

#include <sdrone_motors_task.h>
#include <sdrone_rc_task.h>
#include <sdrone_imu_task.h>

// max 70 deg for roll
#define SDRONE_MAX_ROLL_RADIANS  1.2217f
#define SDRONE_NORM_THROTTLE_TO_ACCEL_FACTOR ((float)MOTORS_ACCEL_RANGE/(float)SDRONE_RC_CHANNEL_RANGE)
#define SDRONE_NORM_ROLL_TO_RADIANS_FACTOR ((float)2.0*SDRONE_MAX_ROLL_RADIANS/(float)SDRONE_RC_CHANNEL_RANGE)

// 10Hz RC frequency
#define SDRONE_REF_SIGNAL_FREQ 10.0f
#define SDRONE_REF_SIGNAL_DT (1.0/SDRONE_REF_SIGNAL_FREQ)
#define SDRONE_AXIS_LENGTH 0.20f
#define SDRONE_TETA_POS    0
#define SDRONE_OMEGA_POS   1
#define SDRONE_ALFA_POS    2
#define SDRONE_THRUST_POS  1


typedef enum {
	SDRONE_MOTORS_DRIVER_ID = 1,
	SDRONE_RC_DRIVER_ID,
	SDRONE_IMU_DRIVER_ID,
	SDRONE_CONTROLLER_DRIVER_ID
} sdrone_drivers_id;

#ifdef MOTORS_FRAME_HORIZONTAL_HEXACOPTER
#define SDRONE_NUM_MOTORS 6
#else
#ifdef MOTORS_FRAME_TWO_HORIZONTAL_AXIS
#define SDRONE_NUM_MOTORS 2
typedef struct {
	float X[3]; // [teta, omega, alfa] (radians)
	float U[2]; // [teta, thrust] (radians, newton)
} sdrone_controller_t;
#endif
#endif

typedef struct {
	sdrone_motors_state_t motors_state;
	sdrone_rc_state_t rc_state;
	sdrone_imu_state_t imu_state;
	uint32_t driver_id;
	sdrone_controller_t controller_state;
} sdrone_state_t;

typedef sdrone_state_t* sdrone_state_handle_t;

void sdrone_controller_task(void *arg);

#endif /* MAIN_SDRONE_CONTROLLER_TASK_H_ */
