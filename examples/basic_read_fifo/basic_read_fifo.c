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

/* Board drivers */
#include "system_interface.h"

/* std */
#include <stdio.h>

/*
 * This example showcases how to retrieve data from the FIFO using the basic driver.
 * It prints accelerometer and/or gyroscope data in either SI unit or in raw data format.
 * High-resolution mode can be enabled and sensors Low-Power/Low-Noise can be configured.
 * It starts accelerometer and gyroscope in Low-Noise mode at 50Hz by default.
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

/* Static variables for command interface */
static uint8_t print_si; /* Indicates if data should be printed in SI */
static uint8_t print_lsb; /* Indicates if data should be printed in LSB */
static uint8_t accel_en; /* Indicates accel state */
static uint8_t gyro_en; /* Indicates gyro state */
static uint8_t hires_en; /* Indicates hires state */
static uint8_t use_ln; /* Indicates which power mode is used (1: LN, 0: LP)*/
static uint8_t discard_accel_samples; /* Indicates how many accel samples should be discarded */
static uint8_t discard_gyro_samples; /* Indicates how many gyro samples should be discarded */

/* Static functions definition */
static int  setup_mcu();
static int  setup_imu();
static int  configure_accel(uint8_t en);
static int  configure_gyro(uint8_t en);
static int  configure_hires(uint8_t en);
static void int_cb(void *context, unsigned int int_num);
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
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example Read FIFO (using basic API)");
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
	int1_timestamp = 0;
	int1_flag      = 0;

	do {
		/* Poll device for data */
		if (int1_flag) {
			uint64_t            timestamp;
			uint16_t            frame_count;
			inv_imu_int_state_t int_state;

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
				rc |= inv_imu_get_frame_count(&imu_dev, &frame_count);
				SI_CHECK_RC(rc);

				for (uint16_t i = 0; i < frame_count; i++) {
					inv_imu_fifo_data_t d;
					char                accel_str[40];
					char                gyro_str[40];
					char                temp_str[20];
					char                fifo_time_str[30];
					int                 accel_raw[3]   = { 0 };
					int                 gyro_raw[3]    = { 0 };
					int                 temp_raw       = 0;
					uint16_t            fifo_timestamp = 0;
					float               accel_g[3]     = { 0 };
					float               gyro_dps[3]    = { 0 };
					float               temp_degc      = 0;
					/*
					 * In hires mode, accel and gyro data are coded as 20-bits signed (max_lsb =
					 * 2^(20-1) = 524288) with max FSR (32 g and 4000 dps). Temperature is coded as
					 * 16-bits signed with a scale factor of 128 and an offset of 25 °C.
					 * Otherwise, accel and gyro data are coded as 16-bits signed 
					 * (max_lsb = 2^(16-1) * = 32768) with the configured FSR (4 g and 2000 dps, 
					 * see `setup_imu()` function).
					 * Temperature is coded as 8-bits signed with a scale factor of 2 and an 
					 * offset of 25 °C.
					 */
					int max_lsb           = hires_en ? 524288 : 32768;
					int accel_fsr_g       = hires_en ? 32 : 4;
					int gyro_fsr_dps      = hires_en ? 4000 : 2000;
					int temp_scale_factor = hires_en ? 128 : 2;
					int temp_offset_c     = 25;

					uint8_t is_accel_data_valid = 0;
					uint8_t is_gyro_data_valid  = 0;
					uint8_t is_temp_data_valid  = 0;

					rc |= inv_imu_get_fifo_frame(&imu_dev, &d);
					SI_CHECK_RC(rc);

					/* Extracts raw data from FIFO data */
					if (hires_en) {
						/* 20 bytes packet */
						accel_raw[0] = d.byte_20.accel_data[0];
						accel_raw[1] = d.byte_20.accel_data[1];
						accel_raw[2] = d.byte_20.accel_data[2];
						if ((discard_accel_samples == 0) &&
						    (accel_raw[0] >> 4 != INVALID_VALUE_FIFO) &&
						    (accel_raw[1] >> 4 != INVALID_VALUE_FIFO) &&
						    (accel_raw[2] >> 4 != INVALID_VALUE_FIFO))
							is_accel_data_valid = 1;

						gyro_raw[0] = d.byte_20.gyro_data[0];
						gyro_raw[1] = d.byte_20.gyro_data[1];
						gyro_raw[2] = d.byte_20.gyro_data[2];
						if ((discard_gyro_samples == 0) &&
						    (gyro_raw[0] >> 4 != INVALID_VALUE_FIFO) &&
						    (gyro_raw[1] >> 4 != INVALID_VALUE_FIFO) &&
						    (gyro_raw[2] >> 4 != INVALID_VALUE_FIFO))
							is_gyro_data_valid = 1;

						temp_raw = d.byte_20.temp_data;
						if (temp_raw >> 4 != INVALID_VALUE_FIFO_1B)
							is_temp_data_valid = 1;

						fifo_timestamp = d.byte_20.timestamp;
					} else if (accel_en && gyro_en) {
						/* 16 bytes packet */
						accel_raw[0] = d.byte_16.accel_data[0];
						accel_raw[1] = d.byte_16.accel_data[1];
						accel_raw[2] = d.byte_16.accel_data[2];
						if ((discard_accel_samples == 0) && (accel_raw[0] != INVALID_VALUE_FIFO) &&
						    (accel_raw[1] != INVALID_VALUE_FIFO) &&
						    (accel_raw[2] != INVALID_VALUE_FIFO))
							is_accel_data_valid = 1;

						gyro_raw[0] = d.byte_16.gyro_data[0];
						gyro_raw[1] = d.byte_16.gyro_data[1];
						gyro_raw[2] = d.byte_16.gyro_data[2];
						if ((discard_gyro_samples == 0) && (gyro_raw[0] != INVALID_VALUE_FIFO) &&
						    (gyro_raw[1] != INVALID_VALUE_FIFO) &&
						    (gyro_raw[2] != INVALID_VALUE_FIFO))
							is_gyro_data_valid = 1;

						temp_raw = (int16_t)d.byte_16.temp_data;
						if (temp_raw != INVALID_VALUE_FIFO_1B)
							is_temp_data_valid = 1;

						fifo_timestamp = d.byte_16.timestamp;
					} else if (accel_en) {
						/* 8 bytes packet with accel only */
						accel_raw[0] = d.byte_8.sensor_data[0];
						accel_raw[1] = d.byte_8.sensor_data[1];
						accel_raw[2] = d.byte_8.sensor_data[2];
						if ((discard_accel_samples == 0) && (accel_raw[0] != INVALID_VALUE_FIFO) &&
						    (accel_raw[1] != INVALID_VALUE_FIFO) &&
						    (accel_raw[2] != INVALID_VALUE_FIFO))
							is_accel_data_valid = 1;

						temp_raw = (int16_t)d.byte_8.temp_data;
						if (temp_raw != INVALID_VALUE_FIFO_1B)
							is_temp_data_valid = 1;

						fifo_timestamp = 0;
					} else {
						/* 8 bytes packet with gyro only */
						gyro_raw[0] = d.byte_8.sensor_data[0];
						gyro_raw[1] = d.byte_8.sensor_data[1];
						gyro_raw[2] = d.byte_8.sensor_data[2];
						if ((discard_gyro_samples == 0) && (gyro_raw[0] != INVALID_VALUE_FIFO) &&
						    (gyro_raw[1] != INVALID_VALUE_FIFO) &&
						    (gyro_raw[2] != INVALID_VALUE_FIFO))
							is_gyro_data_valid = 1;

						temp_raw = (int16_t)d.byte_8.temp_data;
						if (temp_raw != INVALID_VALUE_FIFO_1B)
							is_temp_data_valid = 1;

						fifo_timestamp = 0;
					}

					if ((hires_en || accel_en) && discard_accel_samples)
						discard_accel_samples--;

					if ((hires_en || gyro_en) && discard_gyro_samples)
						discard_gyro_samples--;

					/* Convert data to SI units */
					accel_g[0]  = (float)(accel_raw[0] * accel_fsr_g) / max_lsb;
					accel_g[1]  = (float)(accel_raw[1] * accel_fsr_g) / max_lsb;
					accel_g[2]  = (float)(accel_raw[2] * accel_fsr_g) / max_lsb;
					gyro_dps[0] = (float)(gyro_raw[0] * gyro_fsr_dps) / max_lsb;
					gyro_dps[1] = (float)(gyro_raw[1] * gyro_fsr_dps) / max_lsb;
					gyro_dps[2] = (float)(gyro_raw[2] * gyro_fsr_dps) / max_lsb;
					temp_degc   = (float)temp_offset_c + ((float)temp_raw / temp_scale_factor);

					if (accel_en && gyro_en)
						snprintf(fifo_time_str, 30, "FIFO Time: %5hu us", fifo_timestamp);
					else
						snprintf(fifo_time_str, 30, "FIFO Time:     -");

					/* Print data in SI units */
					if (print_si) {
						if (is_accel_data_valid)
							snprintf(accel_str, 40, "Accel:% 8.2f % 8.2f % 8.2f g", accel_g[0],
							         accel_g[1], accel_g[2]);
						else
							snprintf(accel_str, 40, "Accel:       -        -        -  ");

						if (is_gyro_data_valid)
							snprintf(gyro_str, 40, "Gyro:% 8.2f % 8.2f % 8.2f dps", gyro_dps[0],
							         gyro_dps[1], gyro_dps[2]);
						else
							snprintf(gyro_str, 40, "Gyro:       -        -        -    ");

						if (is_temp_data_valid)
							snprintf(temp_str, 20, "Temp: % 4.2f degC", temp_degc);
						else
							snprintf(temp_str, 20, "Temp:  N/A  ");

						INV_MSG(INV_MSG_LEVEL_INFO, "SI  %10llu us   %s   %s   %s   %s", timestamp,
						        accel_str, gyro_str, temp_str, fifo_time_str);
					}

					/* Print LSB data. */
					if (print_lsb) {
						if (is_accel_data_valid)
							snprintf(accel_str, 40, "Accel:% 8d % 8d % 8d", accel_raw[0],
							         accel_raw[1], accel_raw[2]);
						else
							snprintf(accel_str, 40, "Accel:       -        -        -");

						if (is_gyro_data_valid)
							snprintf(gyro_str, 40, "Gyro:% 8d % 8d % 8d", gyro_raw[0], gyro_raw[1],
							         gyro_raw[2]);
						else
							snprintf(gyro_str, 40, "Gyro:       -        -        -");

						if (is_temp_data_valid)
							snprintf(temp_str, 20, "Temp: % 6d", temp_raw);
						else
							snprintf(temp_str, 20, "Temp:  N/A  ");

						INV_MSG(INV_MSG_LEVEL_INFO, "LSB %10llu us   %s     %s       %s        %s",
						        timestamp, accel_str, gyro_str, temp_str, fifo_time_str);
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
	int                      rc     = 0;
	uint8_t                  whoami = 0;
	inv_imu_int_pin_config_t int_pin_config;
	inv_imu_int_state_t      int_config;
	inv_imu_fifo_config_t    fifo_config;

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
	rc |= inv_imu_get_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);
	fifo_config.gyro_en    = gyro_en;
	fifo_config.accel_en   = accel_en;
	fifo_config.hires_en   = hires_en;
	fifo_config.fifo_wm_th = 1;
	fifo_config.fifo_mode  = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT;
	rc |= inv_imu_set_fifo_config(&imu_dev, &fifo_config);
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

	/* Set power modes */
	if (use_ln) {
		if (accel_en)
			rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
		if (gyro_en)
			rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LN);
	} else {
		if (accel_en)
			rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LP);
		if (gyro_en)
			rc |= inv_imu_set_gyro_mode(&imu_dev, PWR_MGMT0_GYRO_MODE_LP);
	}

	/* Discard N samples at 50Hz to ignore samples at sensor enabling time */
	if (accel_en)
		discard_accel_samples = (ACC_STARTUP_TIME_US / 20000) + 1;
	if (gyro_en)
		discard_gyro_samples = (GYR_STARTUP_TIME_US / 20000) + 1;

	SI_CHECK_RC(rc);

	return rc;
}

/* Configure FIFO and power mode for accel */
static int configure_accel(uint8_t en)
{
	int                    rc = 0;
	inv_imu_fifo_config_t  fifo_config;
	pwr_mgmt0_accel_mode_t accel_mode;

	rc |= inv_imu_get_fifo_config(&imu_dev, &fifo_config);
	fifo_config.accel_en = en;
	rc |= inv_imu_set_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);

	/* Discard N samples at 50Hz to ignore samples at sensor enabling time */
	if (en && !accel_en)
		discard_accel_samples = (ACC_STARTUP_TIME_US / 20000) + 1;

	if (en) {
		if (use_ln)
			accel_mode = PWR_MGMT0_ACCEL_MODE_LN;
		else
			accel_mode = PWR_MGMT0_ACCEL_MODE_LP;
	} else {
		accel_mode = PWR_MGMT0_ACCEL_MODE_OFF;
	}
	rc |= inv_imu_set_accel_mode(&imu_dev, accel_mode);
	SI_CHECK_RC(rc);

	return 0;
}

/* Configure FIFO and power mode for gyro */
static int configure_gyro(uint8_t en)
{
	int                   rc = 0;
	inv_imu_fifo_config_t fifo_config;
	pwr_mgmt0_gyro_mode_t gyro_mode;

	rc |= inv_imu_get_fifo_config(&imu_dev, &fifo_config);
	fifo_config.gyro_en = en;
	rc |= inv_imu_set_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);

	/* Discard N samples at 50Hz to ignore samples at sensor enabling time */
	if (en && !gyro_en)
		discard_gyro_samples = (GYR_STARTUP_TIME_US / 20000) + 1;

	if (en) {
		if (use_ln)
			gyro_mode = PWR_MGMT0_GYRO_MODE_LN;
		else
			gyro_mode = PWR_MGMT0_GYRO_MODE_LP;
	} else {
		gyro_mode = PWR_MGMT0_GYRO_MODE_OFF;
	}
	rc |= inv_imu_set_gyro_mode(&imu_dev, gyro_mode);
	SI_CHECK_RC(rc);

	return 0;
}

/* Configure high-resolution mode for FIFO */
static int configure_hires(uint8_t en)
{
	int                   rc = 0;
	inv_imu_fifo_config_t fifo_config;

	rc |= inv_imu_get_fifo_config(&imu_dev, &fifo_config);
	fifo_config.hires_en = en;
	rc |= inv_imu_set_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);

	return 0;
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
