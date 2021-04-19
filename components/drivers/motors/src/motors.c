#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <motors.h>

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
static mcpwm_io_signals_t motors_timers[MOTORS_MAX_NUM] = {
		MCPWM_TIMER_0, MCPWM_TIMER_0, MCPWM_TIMER_1, MCPWM_TIMER_1, MCPWM_TIMER_2, MCPWM_TIMER_2,
		MCPWM_TIMER_0, MCPWM_TIMER_0, MCPWM_TIMER_1, MCPWM_TIMER_1, MCPWM_TIMER_2, MCPWM_TIMER_2
};
static mcpwm_generator_t motors_pwm_generator[MOTORS_MAX_NUM] = {
		MCPWM0A, MCPWM0B, MCPWM1A, MCPWM1B,MCPWM2A, MCPWM2B,
		MCPWM0A, MCPWM0B, MCPWM1A, MCPWM1B,MCPWM2A, MCPWM2B
};

esp_err_t motors_init_pwm_timer(motors_handle_t motors_handle, mcpwm_unit_t unit, mcpwm_io_signals_t timer) {
	printf("motors: motors_init_pwm_timer [%d/%d]\n", unit, timer);
    mcpwm_config_t pwm_config;
    pwm_config.frequency = motors_handle->frequency;
    // FIXME: cmpr_a/b dipendono dalla frequenza. Aggiungere calcolo
    pwm_config.cmpr_a = 40.0;
    pwm_config.cmpr_b = 40.0;
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    return mcpwm_init(unit, timer, &pwm_config);
}

esp_err_t motors_low_duty_motor(motors_handle_t motors_handle, uint8_t motor_num) {
	printf("motors: motors_low_duty_motor [%d]\n", motor_num);
	ESP_ERROR_CHECK(mcpwm_set_duty(motors_handle->motor[motor_num].mcpwm, motors_timers[motor_num],motors_pwm_generator[motor_num],40.0));
	return ESP_OK;
}

esp_err_t motors_high_duty_motor(motors_handle_t motors_handle, uint8_t motor_num) {
	printf("motors: motors_high_duty_motor [%d]\n", motor_num);
	ESP_ERROR_CHECK(mcpwm_set_duty(motors_handle->motor[motor_num].mcpwm, motors_timers[motor_num],motors_pwm_generator[motor_num],98.0));
	return ESP_OK;
}

esp_err_t motors_low_duty_motors(motors_handle_t motors_handle) {
	printf("motors: motors_low_duty_motors\n");
	for (uint8_t i = 0; i < MOTORS_MAX_NUM; i++) {
		if (motors_handle->motor[i].enabled) {
			ESP_ERROR_CHECK(motors_low_duty_motor(motors_handle, i));
		}
	}
	return ESP_OK;
}

esp_err_t motors_high_duty_motors(motors_handle_t motors_handle) {
	printf("motors: motors_high_duty_motors\n");
	for (uint8_t i = 0; i < MOTORS_MAX_NUM; i++) {
		if (motors_handle->motor[i].enabled) {
			ESP_ERROR_CHECK(motors_high_duty_motor(motors_handle, i));
		}
	}
	return ESP_OK;
}
esp_err_t motors_update_motors(motors_handle_t motors_handle) {
	printf("motors: motors_update_motors\n");
	if(motors_handle->status == ON_ARMED) {
	  for (uint8_t i = 0; i < MOTORS_MAX_NUM; i++) {
		if (motors_handle->motor[i].enabled) {
		  printf("motors: motors_update_motors[%d/%2.2f]\n", i, motors_handle->motor[i].duty_cycle);
		  ESP_ERROR_CHECK(mcpwm_set_duty(motors_handle->motor[i].mcpwm, motors_timers[i],motors_pwm_generator[i],motors_handle->motor[i].duty_cycle));
		}
	  }
	}
	return ESP_OK;
}
esp_err_t motors_init_mcpwm(motors_handle_t motors_handle, mcpwm_unit_t unit) {
	printf("motors: motors_init_mcpwm [%d]\n", unit);
	uint8_t si = 0;
	if(unit == MCPWM_UNIT_1) {
		si = 6;
	}
    mcpwm_pin_config_t pin_config = {
        .mcpwm0a_out_num = motors_handle->motor[0+si].pin,
        .mcpwm0b_out_num = motors_handle->motor[1+si].pin,
        .mcpwm1a_out_num = motors_handle->motor[2+si].pin,
        .mcpwm1b_out_num = motors_handle->motor[3+si].pin,
        .mcpwm2a_out_num = motors_handle->motor[4+si].pin,
        .mcpwm2b_out_num = motors_handle->motor[5+si].pin,
		// TODO: Vedere se necessari
        .mcpwm_sync0_in_num  = -1,
        .mcpwm_sync1_in_num  = -1,
        .mcpwm_sync2_in_num  = -1,
        .mcpwm_fault0_in_num = -1,
        .mcpwm_fault1_in_num = -1,
        .mcpwm_fault2_in_num = -1,
        .mcpwm_cap0_in_num   = -1,
        .mcpwm_cap1_in_num   = -1,
        .mcpwm_cap2_in_num   = -1
    };
    ESP_ERROR_CHECK(mcpwm_set_pin(unit, &pin_config));

    printf("motors::motors_init_mcpwm: init pwm timers\n");
    ESP_ERROR_CHECK(motors_init_pwm_timer(motors_handle, unit, MCPWM_TIMER_0));
    ESP_ERROR_CHECK(motors_init_pwm_timer(motors_handle, unit, MCPWM_TIMER_1));
    ESP_ERROR_CHECK(motors_init_pwm_timer(motors_handle, unit, MCPWM_TIMER_2));
    printf("motors::motors_init_mcpwm: initialized pwm timers\n");

      // Sync timer 1 and timer 2 with timer 0
    printf("motors::motors_init_mcpwm: sync pwm timers\n");
    ESP_ERROR_CHECK(mcpwm_sync_enable(unit, MCPWM_TIMER_1, 1, 0));
    ESP_ERROR_CHECK(mcpwm_sync_enable(unit, MCPWM_TIMER_2, 1, 0));
    MCPWM[unit]->timer[MCPWM_TIMER_0].sync.out_sel = 1;
    for(uint8_t i = 0; i < 6; i++) {
      if(motors_handle->motor[i+si].enabled) {
        motors_handle->motor[i+si].mcpwm = unit;
        motors_handle->motor[i+si].num = i+si;
        motors_handle->motor[i+si].duty_cycle = 40.0f;
      }
    }
    printf("motors::motors_init_mcpwm: END\n");
    return ESP_OK;
};

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
esp_err_t motors_init(motors_handle_t motors_handle) {
	printf("motors: motors_init\n");
	motors_handle->frequency = MOTORS_PWM_FREQUENCY;
	ESP_ERROR_CHECK(motors_init_mcpwm(motors_handle, MCPWM_UNIT_0));
	ESP_ERROR_CHECK(motors_init_mcpwm(motors_handle, MCPWM_UNIT_1));

	ESP_ERROR_CHECK(motors_switchon(motors_handle));
	return ESP_OK;
}

esp_err_t motors_arm(motors_handle_t motors_handle) {
	printf("motors: motors_arm\n");
	ESP_ERROR_CHECK(motors_switchon(motors_handle));
    motors_handle->status = ON_ARMED;
    ESP_ERROR_CHECK(motors_low_duty_motors(motors_handle));;
	return ESP_OK;
}

esp_err_t motors_disarm(motors_handle_t motors_handle) {
    ESP_ERROR_CHECK(motors_low_duty_motors(motors_handle));;
    motors_handle->status = ON_DISARMED;
	return ESP_OK;
}
esp_err_t motors_switchoff(motors_handle_t motors_handle) {
	printf("motors: motors_switchoff\n");
	ESP_ERROR_CHECK(motors_disarm(motors_handle));
	// TODO: provide PIN for switchoff motors
    motors_handle->status = OFF;
	return ESP_OK;
}
esp_err_t motors_switchon(motors_handle_t motors_handle) {
	printf("motors: motors_switchon\n");
	ESP_ERROR_CHECK(motors_low_duty_motors(motors_handle));
	motors_handle->status = ON_DISARMED;
	return ESP_OK;
}
esp_err_t motors_update(motors_handle_t motors_handle) {
    ESP_ERROR_CHECK(motors_update_motors(motors_handle));;
	return ESP_OK;
}

