/*
 * mcpwm.h
 *
 *  Created on: 23 feb 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_MOTORS_INCLUDE_MOTORS_H_
#define COMPONENTS_MOTORS_INCLUDE_MOTORS_H_
#include <stdint.h>
#include <esp_err.h>
#include <driver/mcpwm.h>
#include <soc/mcpwm_periph.h>

#define MOTORS_MAX_NUM 8

// TODO: Define all positions from quad to opto
typedef enum {
	FRONT,
	REAR,
	LEFT,
	RIGHT,
	FRONT_LEFT,
	FRONT_RIGHT,
	REAR_LEFT,
	REAR_RIGHT
} motor_position_t;

typedef enum {
	ON_DISARMED,
	ON_ARMED,
	OFF
} motors_status_t;

typedef struct {
	uint8_t num;
	uint8_t pin;
	motor_position_t position;
	mcpwm_dev_t* mcpwm;
	float duty_cycle;
} motor_t;

typedef struct {
	motor_t motor[MOTORS_MAX_NUM];
	uint16_t frequency;
	uint8_t motors_num;
	uint8_t switch_on_off_pin;
	motors_status_t status;
} motors_t;

typedef motors_t* motors_handle_t;

esp_err_t motors_init(motors_handle_t motors_handle);
esp_err_t motors_arm(motors_handle_t motors_handle);
esp_err_t motors_disarm(motors_handle_t motors_handle);
esp_err_t motors_switchoff(motors_handle_t motors_handle);
esp_err_t motors_switchon(motors_handle_t motors_handle, bool program_esc);


#endif /* COMPONENTS_MOTORS_INCLUDE_MOTORS_H_ */
