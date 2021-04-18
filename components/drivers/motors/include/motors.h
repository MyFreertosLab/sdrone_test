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

typedef struct {
	uint8_t num;
	uint8_t pin;
	mcpwm_dev_t* mcpwm;
} motor_t;

typedef struct {
	motor_t motor[MOTORS_MAX_NUM];
} motors_t;

#endif /* COMPONENTS_MOTORS_INCLUDE_MOTORS_H_ */
