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

#define SDRONE_NORM_THROTTLE_TO_NEWTON_FACTOR ((float)MOTORS_MAX_NEWTON/(float)SDRONE_RC_CHANNEL_RANGE)

typedef enum {
	SDRONE_MOTORS_DRIVER_ID = 1,
	SDRONE_RC_DRIVER_ID,
	SDRONE_IMU_DRIVER_ID,
	SDRONE_CONTROLLER_DRIVER_ID
} sdrone_drivers_id;

typedef struct {
	sdrone_motors_state_t motors_state;
	sdrone_rc_state_t rc_state;
	sdrone_imu_state_t imu_state;
	uint32_t driver_id;
} sdrone_state_t;

typedef sdrone_state_t* sdrone_state_handle_t;

void sdrone_controller_task(void *arg);

#endif /* MAIN_SDRONE_CONTROLLER_TASK_H_ */
