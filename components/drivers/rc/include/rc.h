/*
 * ppm8.h
 *
 *  Created on: 23 feb 2021
 *      Author: andrea
 */

#ifndef COMPONENTS_RC_INCLUDE_RC_H_
#define COMPONENTS_RC_INCLUDE_RC_H_
#include <stdint.h>
#include <esp_err.h>

#define RC_CHANNEL            0                  /* Channel input (0-7) for receiver */
#define RC_GPIO_NUM           23                 /* GPIO number for receiver */
#define RC_CLK_DIV            8                  /* Clock divider */
#define RC_TICK_US            (80/RC_CLK_DIV)    /* Number of Ticks for us */
#define RC_PPM_TIMEOUT_US     3500               /* min PPM silence (us) */
#define RC_BUFF_BLOCK_NUM     4                  /* Memory Blocks */
#define RC_TICK_TRASH         100                /* Interference */
#define RC_MAX_CHANNELS       8

typedef enum {
        RC_TXRX_IGNORE = 0,
        RC_TXRX_TRANSMITTED = 1,
        RC_TXRX_RECEIVED = 2
} rc_txrx_signal_t;

typedef struct {
    uint8_t channel;
    uint8_t gpio_num;
    uint8_t clock_div;
    uint8_t mem_block_num;
    uint8_t ticks_thresh;
    uint16_t idle_threshold;
} rc_config_t;
typedef rc_config_t* rc_config_handle_t;
typedef struct {
	uint32_t channels[RC_MAX_CHANNELS];
	volatile rc_txrx_signal_t txrx_signal;
} rc_data_t;
typedef struct {
	rc_config_t config;
	rc_data_t data;
} rc_t;
typedef rc_t* rc_handle_t;

esp_err_t rc_init(rc_handle_t rc_handle);
esp_err_t rc_start(rc_handle_t rc_handle);
esp_err_t rc_stop(rc_handle_t rc_handle);

#endif /* COMPONENTS_RC_INCLUDE_RC_H_ */
