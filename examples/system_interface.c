/*
 *
 * Copyright (c) [2020] by InvenSense, Inc.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "system_interface.h"

/* Standard includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* IMU drivers */
#include "imu/inv_imu_defs.h" /* For error codes */

/* 
 * Board 
 */
int si_board_init(void)
{
	return 0;
}

/* 
 * UART 
 */
int si_config_uart_for_print(inv_uart_num_t id, int level)
{
	(void)id;
	(void)level;
	return 0;
}

int si_config_uart_for_bin(inv_uart_num_t id)
{
	(void)id;
	return 0;
}

int si_get_uart_command(inv_uart_num_t id, char *cmd)
{
	(void)id;
	(void)cmd;
	return 0;
}

/* 
 * I/O for IMU device 
 */
int si_io_imu_init(inv_imu_serif_type_t serif_type)
{
	(void)serif_type;
	return 0;
}

int si_io_imu_read_reg(uint8_t reg, uint8_t *buf, uint32_t len)
{
	(void)reg;
	(void)buf;
	(void)len;
	return 0;
}

int si_io_imu_write_reg(uint8_t reg, const uint8_t *buf, uint32_t len)
{
	(void)reg;
	(void)buf;
	(void)len;
	return 0;
}

/* 
 * Timers 
 */
int si_init_timers()
{
	return 0;
}

void si_sleep_us(uint32_t us)
{
	(void)us;
}

uint64_t si_get_time_us()
{
	return 0;
}

/*
 * GPIO
 */
int si_init_gpio_int(unsigned int_num, void (*int_cb)(void *context, unsigned int_num))
{
	(void)int_num;
	(void)int_cb;
	return 0;
}

int si_start_gpio_fsync(uint32_t freq, void (*fsync_timer_cb)(void *context))
{
	(void)freq;
	(void)fsync_timer_cb;
	return 0;
}

int si_stop_gpio_fsync()
{
	return 0;
}

void si_toggle_gpio_fsync(void)
{
}

/*
 * Common
 */

void si_disable_irq()
{
}

void si_enable_irq()
{
}

/*
 * Error codes
 */
int si_print_error_if_any(int rc)
{
	if (rc != 0) {
		switch (rc) {
		case INV_IMU_ERROR:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Unspecified error (%d)", rc);
			break;
		case INV_IMU_ERROR_TRANSPORT:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Error occurred at transport level (%d)", rc);
			break;
		case INV_IMU_ERROR_TIMEOUT:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Action did not complete in the expected time window (%d)",
			        rc);
			break;
		case INV_IMU_ERROR_BAD_ARG:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Invalid argument provided (%d)", rc);
			break;
		case INV_IMU_ERROR_EDMP_BUF_EMPTY:
			INV_MSG(INV_MSG_LEVEL_ERROR, "EDMP buffer is empty (%d)", rc);
			break;
		default:
			INV_MSG(INV_MSG_LEVEL_ERROR, "Unknown error (%d)", rc);
			break;
		}
	}

	return rc;
}

/*
 * Message
 */
void inv_msg(int level, const char *str, ...)
{
	(void)level;
	(void)str;
}
