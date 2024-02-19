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

/* Driver */
#include "imu/inv_imu_driver.h"
#include "imu/inv_imu_selftest.h"

/* Board drivers */
#include "system_interface.h"

/* std */
#include <stdio.h>

/*
 * This example showcases how to run selftest and outputs selftest status.
 * Selftest for accelerometer and gyroscope can be enabled independently through UART.
 * Selftest procedure is run once every second. 
 */

/*
 * Select communication link between SmartMotion and IMU.
 * SPI4: `UI_SPI4`
 * I2C:  `UI_I2C`
 */
#define SERIF_TYPE UI_SPI4

/* Static variables */
static inv_imu_device_t              imu_dev; /* Driver structure */
static inv_imu_selftest_parameters_t st_params;

/* Static functions definition */
static int setup_mcu();
static int setup_imu();
static int get_uart_command();
static int print_help();
static int print_current_config();

/* Main function implementation */
int main(void)
{
	int      rc                   = 0;
	uint64_t last_stc_run_time_us = 0;

	rc |= setup_mcu();
	SI_CHECK_RC(rc);

	INV_MSG(INV_MSG_LEVEL_INFO, "###");
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example SELFTEST");
	INV_MSG(INV_MSG_LEVEL_INFO, "###");

	/* Reset commands interface states */

	rc |= setup_imu();
	SI_CHECK_RC(rc);

	rc |= inv_imu_selftest_init_params(&imu_dev, &st_params);
	rc |= print_current_config();
	SI_CHECK_RC(rc);

	do {
		/* Run every second selftest procedure */
		if ((si_get_time_us() - last_stc_run_time_us) >= 1000 * 1000) {
			if (st_params.gyro_en == 0 && st_params.accel_en == 0) {
				INV_MSG(INV_MSG_LEVEL_INFO,
				        "Nothing to execute (accel and gyro self-test disabled)");
			} else {
				inv_imu_selftest_output_t st_output;
				INV_MSG(INV_MSG_LEVEL_INFO, "Running Selftest...");
				rc |= inv_imu_selftest(&imu_dev, &st_params, &st_output);
				last_stc_run_time_us = si_get_time_us();

				if (rc < 0) {
					INV_MSG(INV_MSG_LEVEL_ERROR, "An error occured while running selftest");
				} else {
					if (st_output.gyro_status == INV_IMU_ST_STATUS_SUCCESS)
						INV_MSG(INV_MSG_LEVEL_INFO, "Gyro Selftest PASS");
					else if (st_output.gyro_status == INV_IMU_ST_STATUS_NOT_RUN)
						INV_MSG(INV_MSG_LEVEL_INFO, "Gyro Selftest NOT RUN");
					else {
						INV_MSG(INV_MSG_LEVEL_INFO, "Gyro Selftest FAIL");
						if (st_output.gx_status != INV_IMU_ST_STATUS_SUCCESS)
							INV_MSG(INV_MSG_LEVEL_INFO, "X axis FAIL");
						if (st_output.gy_status != INV_IMU_ST_STATUS_SUCCESS)
							INV_MSG(INV_MSG_LEVEL_INFO, "Y axis FAIL");
						if (st_output.gz_status != INV_IMU_ST_STATUS_SUCCESS)
							INV_MSG(INV_MSG_LEVEL_INFO, "Z axis FAIL");
					}

					if (st_output.accel_status == INV_IMU_ST_STATUS_SUCCESS)
						INV_MSG(INV_MSG_LEVEL_INFO, "Accel Selftest PASS");
					else if (st_output.accel_status == INV_IMU_ST_STATUS_NOT_RUN)
						INV_MSG(INV_MSG_LEVEL_INFO, "Accel Selftest NOT RUN");
					else {
						INV_MSG(INV_MSG_LEVEL_INFO, "Accel Selftest FAIL");
						if (st_output.ax_status != INV_IMU_ST_STATUS_SUCCESS)
							INV_MSG(INV_MSG_LEVEL_INFO, "X axis FAIL");
						if (st_output.ay_status != INV_IMU_ST_STATUS_SUCCESS)
							INV_MSG(INV_MSG_LEVEL_INFO, "Y axis FAIL");
						if (st_output.az_status != INV_IMU_ST_STATUS_SUCCESS)
							INV_MSG(INV_MSG_LEVEL_INFO, "Z axis FAIL");
					}
				}
			}
		}

		rc |= get_uart_command();
	} while (rc == 0);

	return rc;
}

/* Initializes MCU peripherals. */
static int setup_mcu()
{
	int rc = 0;

	rc |= si_board_init();

	/* Configure UART for log */
	rc |= si_config_uart_for_print(SI_UART_ID_FTDI, INV_MSG_LEVEL_DEBUG);

	/* Init timer peripheral for sleep and get_time */
	rc |= si_init_timers();

	/* Initialize serial interface between MCU and IMU */
	rc |= si_io_imu_init(SERIF_TYPE);

	return rc;
}

/* Initializes IMU device and apply configuration. */
static int setup_imu()
{
	int     rc     = 0;
	uint8_t whoami = 0;

	/* Init transport layer */
	imu_dev.transport.read_reg   = si_io_imu_read_reg;
	imu_dev.transport.write_reg  = si_io_imu_write_reg;
	imu_dev.transport.serif_type = SERIF_TYPE;
	imu_dev.transport.sleep_us   = si_sleep_us;

	/* Wait 3 ms to ensure device is properly supplied  */
	si_sleep_us(3000);

	/* In SPI, configure slew-rate to prevent bus corruption on DK-SMARTMOTION-REVG */
	if (imu_dev.transport.serif_type == UI_SPI3 || imu_dev.transport.serif_type == UI_SPI4) {
		drive_config0_t drive_config0;
		drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
		rc |= inv_imu_write_reg(&imu_dev, DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);
		SI_CHECK_RC(rc);
		si_sleep_us(2); /* Takes effect 1.5 us after the register is programmed */
	}

	/* Check whoami */
	rc |= inv_imu_get_who_am_i(&imu_dev, &whoami);
	SI_CHECK_RC(rc);
	if (whoami != INV_IMU_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Erroneous WHOAMI value.");
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Read 0x%02x", whoami);
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Expected 0x%02x", INV_IMU_WHOAMI);
		return -1;
	}

	rc |= inv_imu_soft_reset(&imu_dev);
	SI_CHECK_RC(rc);

	return rc;
}

/* Get command from user through UART */
static int get_uart_command()
{
	int  rc  = 0;
	char cmd = 0;

	rc |= si_get_uart_command(SI_UART_ID_FTDI, &cmd);
	SI_CHECK_RC(rc);

	switch (cmd) {
	case 'a':
		st_params.accel_en = !st_params.accel_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "Selftest accel %s.",
		        st_params.accel_en ? "Enabled" : "Disabled");
		break;
	case 'g':
		st_params.gyro_en = !st_params.gyro_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "Selftest gyro %s.",
		        st_params.gyro_en ? "Enabled" : "Disabled");
		break;
	case 'c':
		rc |= print_current_config();
		break;
	case 'h':
	case 'H':
		rc |= print_help();
		break;
	case 0:
		break; /* No command received */
	default:
		INV_MSG(INV_MSG_LEVEL_INFO, "Unknown command : %c", cmd);
		rc |= print_help();
		break;
	}

	return rc;
}

/* Help for UART command interface */
static int print_help()
{
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Help");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'a' : Toggle accel selftest enable");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'g' : Toggle gyro selftest enable");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'c' : Print current configuration");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'h' : Print this helper");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	return 0;
}

/* Print current sample configuration */
static int print_current_config()
{
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Current configuration");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Selftest accel %s.",
	        st_params.accel_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Selftest gyro %s.", st_params.gyro_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	return 0;
}