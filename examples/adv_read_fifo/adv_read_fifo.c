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
#include "imu/inv_imu_driver_advanced.h"

/* Board drivers */
#include "system_interface.h"

/* std */
#include <stdio.h>

/*
 * This example showcases how to retrieve data from the FIFO using the advanced driver.
 * It prints accelerometer and/or gyroscope data in either SI unit or in raw data format.
 * High-resolution mode can be enabled and sensors Low-Power/Low-Noise can be configured.
 * It starts accelerometer and gyroscope in Low-Noise mode at 50Hz by default.

 * It also shows how to handle following issue listed in AN-000364 :
 * When operating in FIFO streaming mode, if FIFO threshold interrupt is triggered with M number
 * of FIFO frames accumulated in the FIFO buffer, the host should only read the first M-1 number
 * of FIFO frames. This prevents the FIFO empty event, that can cause FIFO data corruption, from
 * happening.
 */

/*
 * Select communication link between SmartMotion and IMU.
 * SPI4: `UI_SPI4`
 * I2C:  `UI_I2C`
 */
#define SERIF_TYPE UI_SPI4

/* Static variables */
static inv_imu_device_t  imu_dev; /* Driver structure */
static volatile int      int1_flag; /* Flag set when INT1 is received */
static volatile uint64_t int1_timestamp; /* Store timestamp when int from IMU fires */
static uint8_t           fifo_data[FIFO_MIRRORING_SIZE]; /* Memory where to store FIFO data */
static uint64_t          timestamp; /* Interrupt timestamp to be used in callback */
static uint64_t          accel_start_time_us; /* To discard samples when enabling accel */
static uint64_t          gyro_start_time_us; /* To discard samples when enabling gyro */

/* Static variables for command interface */
static uint8_t print_si; /* Indicates if data should be printed in SI */
static uint8_t print_lsb; /* Indicates if data should be printed in LSB */
static uint8_t accel_en; /* Indicates accel state */
static uint8_t gyro_en; /* Indicates gyro state */
static uint8_t hires_en; /* Indicates hires state */
static uint8_t use_ln; /* Indicates which power mode is used (1: LN, 0: LP)*/

/* Static functions definition */
static int  setup_mcu();
static int  setup_imu();
static int  configure_accel(uint8_t en);
static int  configure_gyro(uint8_t en);
static int  configure_hires(uint8_t en);
static void int_cb(void *context, unsigned int int_num);
static void sensor_event_cb(inv_imu_sensor_event_t *event);
static int  get_uart_command();
static int  print_help();
static int  print_current_config();

/* Main function implementation */
int main(void)
{
	int rc = 0;

	rc |= setup_mcu();
	SI_CHECK_RC(rc);

	INV_MSG(INV_MSG_LEVEL_INFO, "###");
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example Read FIFO (using advanced API)");
	INV_MSG(INV_MSG_LEVEL_INFO, "###");

	/* Reset commands interface states */
	print_si  = 1;
	print_lsb = 0;
	accel_en  = 1;
	gyro_en   = 1;
	hires_en  = 0;
	use_ln    = 1;

	rc |= setup_imu();
	SI_CHECK_RC(rc);

	/* Reset timestamp and interrupt flag */
	int1_flag      = 0;
	int1_timestamp = 0;

	do {
		/* Poll device for data */
		if (int1_flag) {
			inv_imu_int_state_t int_state;
			uint16_t            fifo_count;

			si_disable_irq();
			/* Clear interrupt flag */
			int1_flag = 0;
			/* Retrieve timestamp */
			timestamp = int1_timestamp;
			si_enable_irq();

			/* Read interrupt status */
			rc |= inv_imu_get_int_status(&imu_dev, INV_IMU_INT1, &int_state);
			SI_CHECK_RC(rc);

			if (int_state.INV_FIFO_THS) {
				rc |= inv_imu_adv_get_data_from_fifo(&imu_dev, fifo_data, &fifo_count);
				SI_CHECK_RC(rc);

				rc |= inv_imu_adv_parse_fifo_data(&imu_dev, fifo_data, fifo_count);
				SI_CHECK_RC(rc);
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

	/* Configure GPIO to call `int_cb` when INT1 fires. */
	rc |= si_init_gpio_int(SI_GPIO_INT1, int_cb);

	/* Init timer peripheral for sleep and get_time */
	rc |= si_init_timers();

	/* Initialize serial interface between MCU and IMU */
	rc |= si_io_imu_init(SERIF_TYPE);

	return rc;
}

/* Initializes IMU device and apply configuration. */
static int setup_imu()
{
	int                       rc = 0;
	inv_imu_int_pin_config_t  int_pin_config;
	inv_imu_int_state_t       int_config;
	inv_imu_adv_fifo_config_t fifo_config;
	inv_imu_adv_var_t *       e = (inv_imu_adv_var_t *)imu_dev.adv_var;

	/* Init transport layer */
	imu_dev.transport.read_reg   = si_io_imu_read_reg;
	imu_dev.transport.write_reg  = si_io_imu_write_reg;
	imu_dev.transport.serif_type = SERIF_TYPE;
	imu_dev.transport.sleep_us   = si_sleep_us;

	/* Init sensor event callback */
	e->sensor_event_cb = sensor_event_cb;

	/* In SPI, configure slew-rate to prevent bus corruption on DK-SMARTMOTION-REVG */
	if (imu_dev.transport.serif_type == UI_SPI3 || imu_dev.transport.serif_type == UI_SPI4) {
		drive_config0_t drive_config0;
		drive_config0.pads_spi_slew = DRIVE_CONFIG0_PADS_SPI_SLEW_TYP_10NS;
		rc |= inv_imu_write_reg(&imu_dev, DRIVE_CONFIG0, 1, (uint8_t *)&drive_config0);
		SI_CHECK_RC(rc);
		si_sleep_us(2); /* Takes effect 1.5 us after the register is programmed */
	}

	rc |= inv_imu_adv_init(&imu_dev);
	SI_CHECK_RC(rc);

	/*
	 * Configure interrupts pins
	 * - Polarity High
	 * - Pulse mode
	 * - Push-Pull drive
	 */
	int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
	int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
	int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
	rc |= inv_imu_set_pin_config_int(&imu_dev, INV_IMU_INT1, &int_pin_config);
	SI_CHECK_RC(rc);

	/* Interrupts configuration */
	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
	int_config.INV_FIFO_THS = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_config);
	SI_CHECK_RC(rc);

	/* FIFO configuration */
	rc |= inv_imu_adv_get_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);
	fifo_config.base_conf.gyro_en  = gyro_en;
	fifo_config.base_conf.accel_en = accel_en;
	fifo_config.base_conf.hires_en = hires_en;
	/* 
	 * AN-000364: When operating in FIFO streaming mode, if FIFO threshold interrupt is triggered
	 * with M number of FIFO frames accumulated in the FIFO buffer, the host should only read the
	 * first M-1 number of FIFO frames.
	 * Therefore, configure FIFO threshold to 2 so we can always read 1 frame.
	 */
	fifo_config.base_conf.fifo_wm_th = 2;
	fifo_config.base_conf.fifo_mode  = FIFO_CONFIG0_FIFO_MODE_STREAM;
	fifo_config.tmst_fsync_en        = INV_IMU_ENABLE;
	fifo_config.fifo_wr_wm_gt_th     = FIFO_CONFIG2_FIFO_WR_WM_EQ_OR_GT_TH;
	rc |= inv_imu_adv_set_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);

	/* Set FSR */
	rc |= inv_imu_set_accel_fsr(&imu_dev, ACCEL_CONFIG0_ACCEL_UI_FS_SEL_4_G);
	rc |= inv_imu_set_gyro_fsr(&imu_dev, GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS);
	SI_CHECK_RC(rc);

	/* Set ODR */
	rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_50_HZ);
	rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_50_HZ);
	SI_CHECK_RC(rc);

	/* Set BW = ODR/4 */
	rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
	rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);
	SI_CHECK_RC(rc);

	/* Enable sensors */
	if (use_ln) {
		if (accel_en)
			rc |= inv_imu_adv_enable_accel_ln(&imu_dev);
		if (gyro_en)
			rc |= inv_imu_adv_enable_gyro_ln(&imu_dev);
	} else {
		if (accel_en)
			rc |= inv_imu_adv_enable_accel_lp(&imu_dev);
		if (gyro_en)
			rc |= inv_imu_adv_enable_gyro_lp(&imu_dev);
	}
	if (accel_en)
		accel_start_time_us = si_get_time_us();
	if (gyro_en)
		gyro_start_time_us = si_get_time_us();

	SI_CHECK_RC(rc);

	return rc;
}

/* Configure FIFO and power mode for accel */
static int configure_accel(uint8_t en)
{
	int                       rc = 0;
	inv_imu_adv_fifo_config_t fifo_config;

	rc |= inv_imu_adv_get_fifo_config(&imu_dev, &fifo_config);
	fifo_config.base_conf.accel_en = en;
	rc |= inv_imu_adv_set_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);

	if (!accel_en && en)
		accel_start_time_us = si_get_time_us();

	if (en) {
		if (use_ln)
			rc |= inv_imu_adv_enable_accel_ln(&imu_dev);
		else
			rc |= inv_imu_adv_enable_accel_lp(&imu_dev);
	} else {
		rc |= inv_imu_adv_disable_accel(&imu_dev);
	}
	SI_CHECK_RC(rc);

	return rc;
}

/* Configure FIFO and power mode for gyro */
static int configure_gyro(uint8_t en)
{
	int                       rc = 0;
	inv_imu_adv_fifo_config_t fifo_config;

	rc |= inv_imu_adv_get_fifo_config(&imu_dev, &fifo_config);
	fifo_config.base_conf.gyro_en = en;
	rc |= inv_imu_adv_set_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);

	if (!gyro_en && en)
		gyro_start_time_us = si_get_time_us();

	if (en) {
		if (use_ln)
			rc |= inv_imu_adv_enable_gyro_ln(&imu_dev);
		else
			rc |= inv_imu_adv_enable_gyro_lp(&imu_dev);
	} else {
		rc |= inv_imu_adv_disable_gyro(&imu_dev);
	}
	SI_CHECK_RC(rc);

	return rc;
}

/* Configure high-resolution mode for FIFO */
static int configure_hires(uint8_t en)
{
	int                       rc = 0;
	inv_imu_adv_fifo_config_t fifo_config;

	rc |= inv_imu_adv_get_fifo_config(&imu_dev, &fifo_config);
	fifo_config.base_conf.hires_en = en;
	rc |= inv_imu_adv_set_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);

	return rc;
}

/* IMU interrupt handler. */
static void int_cb(void *context, unsigned int int_num)
{
	(void)context;

	if (int_num == SI_GPIO_INT1) {
		int1_timestamp = si_get_time_us();
		int1_flag      = 1;
	}
}

/* Sensor event callback */
static void sensor_event_cb(inv_imu_sensor_event_t *event)
{
	char  accel_str[40];
	char  gyro_str[40];
	char  temp_str[20];
	char  fifo_time_str[30];
	int   accel_raw[3] = { 0 };
	int   gyro_raw[3]  = { 0 };
	float accel_g[3]   = { 0 };
	float gyro_dps[3]  = { 0 };
	float temp_degc    = 0;
	/*
	 * In hires mode, accel and gyro data are coded as 20-bits signed (max_lsb = 2^(20-1) = 524288)
	 * with max FSR (32 g and 4000 dps). Temperature is coded as 16-bits signed with a scale factor
	 * of 128 and an offset of 25 °C.
	 * Otherwise, accel and gyro data are coded as 16-bits signed (max_lsb = 2^(16-1) = 32768)
	 * with the configured FSR (4 g and 2000 dps, see `setup_imu()` function). Temperature is coded
	 * as 8-bits signed with a scale factor of 2 and an offset of 25 °C.
	 */
	int max_lsb           = hires_en ? 524288 : 32768;
	int accel_fsr_g       = hires_en ? 32 : 4;
	int gyro_fsr_dps      = hires_en ? 4000 : 2000;
	int temp_scale_factor = hires_en ? 128 : 2;
	int temp_offset_c     = 25;

	/* Extracts raw data from FIFO data */
	if (hires_en) {
		accel_raw[0] = event->accel[0] << 4 | event->accel_high_res[0];
		accel_raw[1] = event->accel[1] << 4 | event->accel_high_res[1];
		accel_raw[2] = event->accel[2] << 4 | event->accel_high_res[2];
		gyro_raw[0]  = event->gyro[0] << 4 | event->gyro_high_res[0];
		gyro_raw[1]  = event->gyro[1] << 4 | event->gyro_high_res[1];
		gyro_raw[2]  = event->gyro[2] << 4 | event->gyro_high_res[2];
	} else {
		accel_raw[0] = event->accel[0];
		accel_raw[1] = event->accel[1];
		accel_raw[2] = event->accel[2];
		gyro_raw[0]  = event->gyro[0];
		gyro_raw[1]  = event->gyro[1];
		gyro_raw[2]  = event->gyro[2];
	}

	/* Convert data to SI units */
	accel_g[0]  = (float)(accel_raw[0] * accel_fsr_g) / max_lsb;
	accel_g[1]  = (float)(accel_raw[1] * accel_fsr_g) / max_lsb;
	accel_g[2]  = (float)(accel_raw[2] * accel_fsr_g) / max_lsb;
	gyro_dps[0] = (float)(gyro_raw[0] * gyro_fsr_dps) / max_lsb;
	gyro_dps[1] = (float)(gyro_raw[1] * gyro_fsr_dps) / max_lsb;
	gyro_dps[2] = (float)(gyro_raw[2] * gyro_fsr_dps) / max_lsb;
	temp_degc   = (float)temp_offset_c + ((float)event->temperature / temp_scale_factor);

	if (accel_en && gyro_en)
		snprintf(fifo_time_str, 30, "FIFO Time: %5u us", event->timestamp_fsync);
	else
		snprintf(fifo_time_str, 30, "FIFO Time:     -");

	/* Discard samples until startup time has elapsed. */
	if ((event->sensor_mask & (1 << INV_SENSOR_ACCEL)) && (accel_start_time_us != UINT32_MAX)) {
		if ((si_get_time_us() - accel_start_time_us) >= ACC_STARTUP_TIME_US)
			accel_start_time_us = UINT32_MAX;
		else
			event->sensor_mask &= ~(1 << INV_SENSOR_ACCEL);
	}
	if ((event->sensor_mask & (1 << INV_SENSOR_GYRO)) && (gyro_start_time_us != UINT32_MAX)) {
		if ((si_get_time_us() - gyro_start_time_us) >= GYR_STARTUP_TIME_US)
			gyro_start_time_us = UINT32_MAX;
		else
			event->sensor_mask &= ~(1 << INV_SENSOR_GYRO);
	}

	/* Print data in SI units. */
	if (print_si) {
		if (accel_en && (event->sensor_mask & (1 << INV_SENSOR_ACCEL)))
			snprintf(accel_str, 40, "Accel:% 8.2f % 8.2f % 8.2f g", accel_g[0], accel_g[1],
			         accel_g[2]);
		else
			snprintf(accel_str, 40, "Accel:       -        -        -  ");

		if (gyro_en && (event->sensor_mask & (1 << INV_SENSOR_GYRO)))
			snprintf(gyro_str, 40, "Gyro:% 8.2f % 8.2f % 8.2f dps", gyro_dps[0], gyro_dps[1],
			         gyro_dps[2]);
		else
			snprintf(gyro_str, 40, "Gyro:       -        -        -    ");

		snprintf(temp_str, 20, "Temp: % 4.2f degC", temp_degc);

		INV_MSG(INV_MSG_LEVEL_INFO, "SI  %10llu us   %s   %s   %s   %s", timestamp, accel_str,
		        gyro_str, temp_str, fifo_time_str);
	}

	/* Print LSB data. */
	if (print_lsb) {
		if (accel_en && (event->sensor_mask & (1 << INV_SENSOR_ACCEL)))
			snprintf(accel_str, 40, "Accel:% 8d % 8d % 8d", accel_raw[0], accel_raw[1],
			         accel_raw[2]);
		else
			snprintf(accel_str, 40, "Accel:       -        -        -");

		if (gyro_en && (event->sensor_mask & (1 << INV_SENSOR_GYRO)))
			snprintf(gyro_str, 40, "Gyro:% 8d % 8d % 8d", gyro_raw[0], gyro_raw[1], gyro_raw[2]);
		else
			snprintf(gyro_str, 40, "Gyro:       -        -        -");

		snprintf(temp_str, 20, "Temp: % 6d", event->temperature);

		INV_MSG(INV_MSG_LEVEL_INFO, "LSB %10llu us   %s     %s       %s        %s", timestamp,
		        accel_str, gyro_str, temp_str, fifo_time_str);
	}
}

/* Get command from user through UART */
static int get_uart_command()
{
	int  rc  = 0;
	char cmd = 0;

	rc |= si_get_uart_command(SI_UART_ID_FTDI, &cmd);
	SI_CHECK_RC(rc);

	switch (cmd) {
	case 's':
		print_si = !print_si;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s SI print.", print_si ? "Enabling" : "Disabling");
		break;
	case 'l':
		print_lsb = !print_lsb;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s LSB print.", print_lsb ? "Enabling" : "Disabling");
		break;
	case 'a':
		rc |= configure_accel(!accel_en);
		accel_en = !accel_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s Accel.", accel_en ? "Enabling" : "Disabling");
		break;
	case 'g':
		rc |= configure_gyro(!gyro_en);
		gyro_en = !gyro_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s Gyro.", gyro_en ? "Enabling" : "Disabling");
		break;
	case 'i':
		hires_en = !hires_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s high-res mode.", hires_en ? "Enabling" : "Disabling");
		rc |= configure_hires(hires_en);
		break;
	case 'p':
		use_ln = !use_ln;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s selected.", use_ln ? "Low-noise" : "Low-power");
		rc |= configure_accel(accel_en);
		rc |= configure_gyro(gyro_en);
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
	INV_MSG(INV_MSG_LEVEL_INFO, "# 's' : Toggle print data in SI");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'l' : Toggle print data in LSB");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'a' : Toggle accel data");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'g' : Toggle gyro data");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'i' : Toggle High-resolution mode");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'p' : Toggle power mode (low-noise, low-power)");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'c' : Print current configuration");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'h' : Print this helper");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	si_sleep_us(2000000); /* Give user some time to read */

	return 0;
}

/* Print current sample configuration */
static int print_current_config()
{
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Current configuration");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Accel: %s", accel_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Gyro: %s", gyro_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "# High-resolution mode: %s", hires_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Power mode: %s", use_ln ? "Low-noise" : "Low-power");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Data in SI are %s", print_si ? "printed" : "hidden");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Data in LSB are %s", print_lsb ? "printed" : "hidden");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	si_sleep_us(2000000); /* Give user some time to read */

	return 0;
}