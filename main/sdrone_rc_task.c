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

/*
Min/Center/Max values for Sticks
Channel [0] range: [996][1508][2003]
Channel [1] range: [993][1489][2004]
Channel [2] range: [1001][1461][1991]
Channel [3] range: [1003][1483][2010]
Channel [4] range: [1001][1501][2004]
Channel [5] range: [1001][1501][2003]
Channel [6] range: [1001][1001][1001]
Channel [7] range: [1001][1001][1001]
 */
static sdrone_rc_channels_range_t rc_channels_range[RC_MAX_CHANNELS] = {
		{ .min = 996,
		  .center = 1508,
		  .max = 2003
		},
		{ .min = 993,
		  .center = 1489,
		  .max = 2004
		},
		{ .min = 989,
		  .center = 1469,
		  .max = 1979
		},
		{ .min = 1003,
		  .center = 1483,
		  .max = 2010
		},
		{ .min = 1001,
		  .center = 1501,
		  .max = 2004
		},
		{ .min = 1001,
		  .center = 1501,
		  .max = 2003
		},
		{ .min = 1001,
		  .center = 1501,
		  .max = 2003
		},
		{ .min = 1001,
		  .center = 1501,
		  .max = 2003
		}
};


void sdrone_rc_task(void *arg) {
	sdrone_rc_state_handle_t sdrone_rc_state_handle =
			(sdrone_rc_state_handle_t) arg;
	rc_handle_t rc_handle = &(sdrone_rc_state_handle->rc_data);
	rc_t rc_data_local;
	rc_t* rc_data_local_handle = &rc_data_local;

	ESP_ERROR_CHECK(rc_init(rc_data_local_handle));
	sdrone_rc_state_handle->rc_task_handle = xTaskGetCurrentTaskHandle();

	float scale_factor_left[RC_MAX_CHANNELS];
	float scale_factor_right[RC_MAX_CHANNELS];
	int16_t value[RC_MAX_CHANNELS];

/*
channel [0]; scale: [0.97656][1.01010]
channel [1]; scale: [1.00806][0.97087]
channel [2]; scale: [1.08696][0.94340]
channel [3]; scale: [1.04167][0.94877]
channel [4]; scale: [1.00000][0.99404]
channel [5]; scale: [1.00000][0.99602]
channel [6]; scale: [1.00000][0.99602]
channel [7]; scale: [1.00000][0.99602]

 */
	for(uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
		scale_factor_left[i] = 200.0/(rc_channels_range[i].center - rc_channels_range[i].min);
		scale_factor_right[i] = 200.0/(rc_channels_range[i].max - rc_channels_range[i].center);
		printf("channel [%d]; scale: [%2.5f][%2.5f]\n", i, scale_factor_left[i], scale_factor_right[i]);
	}

	while (true) {
		ESP_ERROR_CHECK(rc_start(rc_data_local_handle));
		if (rc_data_local_handle->data.txrx_signal == RC_TXRX_TRANSMITTED) {
			if ((sdrone_rc_state_handle->controller_task_handle != NULL) && (rc_handle->data.txrx_signal != RC_TXRX_TRANSMITTED)) {
				memcpy(rc_handle, rc_data_local_handle,
						sizeof(*rc_data_local_handle));
				xTaskNotify(sdrone_rc_state_handle->controller_task_handle,
						sdrone_rc_state_handle->driver_id,
						eSetValueWithOverwrite);

				// TODO: inviare anche i valori normalizzati oltre che quelli raw
				// TODO: i dati raw possono essere uint16_t. Modificare rc_t
				for(uint8_t i = 0; i < RC_MAX_CHANNELS; i++) {
					if(rc_data_local_handle->data.channels[i] < rc_channels_range[i].center) {
						value[i] = (int16_t)((rc_channels_range[i].center - rc_data_local_handle->data.channels[i])*scale_factor_left[i]);
						value[i] = -value[i];
					} else {
						value[i] = (int16_t)((rc_data_local_handle->data.channels[i] - rc_channels_range[i].center)*scale_factor_right[i]);
					}
					printf("[%d]-v[%d] ", i, value[i]);
				}
				printf("\n");
			}
		}
	}
	vTaskDelete(NULL);

}
