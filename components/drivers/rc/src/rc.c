/*
 * ppm8.c
 *
 *  Created on: 23 feb 2021
 *      Author: andrea
 */
#include <string.h>
#include <rc.h>
#include <driver/gpio.h>
#include "driver/rmt.h"

static 	RingbufHandle_t rb = NULL;

/************************************************************************
 ****************** A P I  I M P L E M E N T A T I O N ******************
 ************************************************************************/
rmt_config_t rmt_rx;
esp_err_t rc_init(rc_handle_t rc_handle) {
	printf("PPM RX initialization started ... \n");
	memset(rc_handle, 0, sizeof(*rc_handle));

	// set configuration
	rc_handle->config.gpio_num = RC_GPIO_NUM;
	rc_handle->config.channel = RC_CHANNEL;
	rc_handle->config.gpio_num = RC_GPIO_NUM;
	rc_handle->config.clock_div = RC_CLK_DIV;
	rc_handle->config.mem_block_num = RC_BUFF_BLOCK_NUM;
	rc_handle->config.ticks_thresh = RC_TICK_TRASH;
	rc_handle->config.idle_threshold = RC_PPM_TIMEOUT_US * (RC_TICK_US);

	// prepare RMT component
	printf("Gpio Pin [%d] ... \n", rc_handle->config.gpio_num);
	gpio_pulldown_en(rc_handle->config.gpio_num);
	gpio_set_direction(rc_handle->config.gpio_num, GPIO_MODE_INPUT);

	printf("Configuring RMT module ... \n");
	rmt_rx.channel = rc_handle->config.channel;
	rmt_rx.gpio_num = rc_handle->config.gpio_num;
	rmt_rx.clk_div = rc_handle->config.clock_div;
	rmt_rx.mem_block_num = rc_handle->config.mem_block_num;
	rmt_rx.rmt_mode = RMT_MODE_RX;
	rmt_rx.rx_config.filter_en = true;
	rmt_rx.rx_config.filter_ticks_thresh = rc_handle->config.ticks_thresh;
	rmt_rx.rx_config.idle_threshold = rc_handle->config.idle_threshold;
	rmt_config(&rmt_rx);

	printf("Installing RMT module ... \n");
	rmt_driver_install(rmt_rx.channel, 1000, 0);
	rmt_get_ringbuf_handle(rc_handle->config.channel, &rb);
	printf("PPM RX initialized\n");
	return ESP_OK;
}

esp_err_t rc_start(rc_handle_t rc_handle) {
	uint16_t rmt_tick_microseconds = (80 / rc_handle->config.clock_div); /*!< RMT counter value for 10 us.(Source clock is APB clock) */
    rc_handle->data.txrx_signal = RC_TXRX_IGNORE;

	rmt_rx_start(rc_handle->config.channel, true);

	uint8_t counter = 0;
	while (rb) {
		size_t rx_size = 0;
		rmt_item32_t *item = (rmt_item32_t*) xRingbufferReceive(rb, &rx_size,
				pdMS_TO_TICKS(25));
		if (item) {
			counter++;
			for (int i = 0; i < RC_MAX_CHANNELS; i++) {
				rc_handle->data.channels[i] = ((item + i)->duration1
						+ (item + i)->duration0) / rmt_tick_microseconds;
			}
            rc_handle->data.txrx_signal = RC_TXRX_TRANSMITTED;
			vRingbufferReturnItem(rb, (void*) item);
			if(counter == 4) {
				break;
			}
		} else {
			printf("RC::Data NOT received\n");
		}
	}
	ESP_ERROR_CHECK(rc_stop(rc_handle));
	return ESP_OK;
}

esp_err_t rc_stop(rc_handle_t rc_handle) {
	return rmt_rx_stop(rc_handle->config.channel);
}
