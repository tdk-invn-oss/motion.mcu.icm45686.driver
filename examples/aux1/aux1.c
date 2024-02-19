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
#include "imu/inv_imu_driver_aux1.h"

/* Board drivers */
#include "system_interface.h"

/* std */
#include <stdio.h>

/*
 * This example showcases how to retrieve OIS data from AUX1 interface.
 * It gathers accelerometer and/or gyroscope data when data ready fires INT2.
 */

/*
 * Only SPI4 is available between SmartMotion and IMU as I2C would be too slow for AUX sensor ODR
 */
#define SERIF_TYPE UI_SPI4

#define SLIDING_AVERAGE(AVG, N, DATA) (float)((AVG * (double)(N - 1) + DATA) / (double)N)

/* Static variables */
static inv_imu_transport_t aux_dev; /* AUX1 Driver structure */
static volatile int        int2_flag; /* Flag set when INT2 is received */
static volatile uint64_t   int2_timestamp; /* Store timestamp when int from IMU fires */

/* Static variables for command interface */
static uint8_t  accel_en; /* Indicates accel state */
static uint8_t  gyro_en; /* Indicates gyro state */
static uint8_t  stats_en; /* Indicates way to print sensor data */
static uint32_t nb_samples_acc; /* Number of accel data read on AUX1 */
static uint32_t nb_samples_gyr; /* Number of gyro data read on AUX1 */
static float    average_acc[3]; /* Average value of accelerometer sensor in g */
static float    average_gyr[3]; /* Average value of gyroscope sensor in dps */

/* Static functions definition */
static int  setup_mcu();
static int  setup_imu();
static int  configure_accel();
static int  configure_gyro();
static void int_cb(void *context, unsigned int int_num);
static int  get_uart_command();
static int  print_help();
static int  print_current_config();

/* Main function implementation */
int main(void)
{
	int      rc              = 0;
	uint64_t last_print_time = si_get_time_us();

	rc |= setup_mcu();
	SI_CHECK_RC(rc);

	INV_MSG(INV_MSG_LEVEL_INFO, "###");
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example AUX1");
	INV_MSG(INV_MSG_LEVEL_INFO, "###");

	/* Reset commands interface states */
	accel_en       = 1;
	gyro_en        = 1;
	stats_en       = 1;
	nb_samples_acc = 0;
	nb_samples_gyr = 0;
	average_acc[0] = 0;
	average_acc[1] = 0;
	average_acc[2] = 0;
	average_gyr[0] = 0;
	average_gyr[1] = 0;
	average_gyr[2] = 0;

	rc |= setup_imu();
	SI_CHECK_RC(rc);

	/* Reset timestamp and interrupt flag */
	int2_flag      = 0;
	int2_timestamp = 0;

	do {
		/* Poll device for data */
		if (int2_flag) {
			uint64_t              timestamp;
			inv_imu_sensor_data_t d;
			char                  accel_str[60];
			char                  gyro_str[60];

			si_disable_irq();
			/* Clear interrupt flag */
			int2_flag = 0;
			/* Retrieve timestamp */
			timestamp = int2_timestamp;
			si_enable_irq();

			rc |= inv_imu_get_aux1_register_data(&aux_dev, &d);
			SI_CHECK_RC(rc);

			/*
			 * Convert data to SI units
			 * Accel and gyro data are coded as 16-bits signed (max_lsb = 2^(16-1) = 32768) with 
			 * the configured FSR (8 g and 2000 dps, see `setup_imu()` function).
			 */

			if (accel_en && d.accel_data[0] != INVALID_VALUE_FIFO &&
			    d.accel_data[1] != INVALID_VALUE_FIFO && d.accel_data[2] != INVALID_VALUE_FIFO) {
				float accel_g[3];
				accel_g[0] = (float)(d.accel_data[0] * 8 /* gee */) / 32768;
				accel_g[1] = (float)(d.accel_data[1] * 8 /* gee */) / 32768;
				accel_g[2] = (float)(d.accel_data[2] * 8 /* gee */) / 32768;

				if (!stats_en)
					snprintf(accel_str, 60, "Accel        :% 8.2f % 8.2f % 8.2f g", accel_g[0],
					         accel_g[1], accel_g[2]);
				else {
					nb_samples_acc++;
					average_acc[0] = SLIDING_AVERAGE(average_acc[0], nb_samples_acc, accel_g[0]);
					average_acc[1] = SLIDING_AVERAGE(average_acc[1], nb_samples_acc, accel_g[1]);
					average_acc[2] = SLIDING_AVERAGE(average_acc[2], nb_samples_acc, accel_g[2]);

					snprintf(accel_str, 60, "Average Accel:% 8.2f % 8.2f % 8.2f g", average_acc[0],
					         average_acc[1], average_acc[2]);
				}
			} else if (!stats_en)
				snprintf(accel_str, 60, "Accel        :       -        -        -  ");
			else
				snprintf(accel_str, 60, "Average Accel:       -        -        -  ");

			if (gyro_en && d.gyro_data[0] != INVALID_VALUE_FIFO &&
			    d.gyro_data[1] != INVALID_VALUE_FIFO && d.gyro_data[2] != INVALID_VALUE_FIFO) {
				float gyro_dps[3];
				gyro_dps[0] = (float)(d.gyro_data[0] * 2000 /* dps */) / 32768;
				gyro_dps[1] = (float)(d.gyro_data[1] * 2000 /* dps */) / 32768;
				gyro_dps[2] = (float)(d.gyro_data[2] * 2000 /* dps */) / 32768;

				if (!stats_en)
					snprintf(gyro_str, 60, "Gyro        :% 8.2f % 8.2f % 8.2f dps", gyro_dps[0],
					         gyro_dps[1], gyro_dps[2]);
				else {
					nb_samples_gyr++;
					average_gyr[0] = SLIDING_AVERAGE(average_gyr[0], nb_samples_gyr, gyro_dps[0]);
					average_gyr[1] = SLIDING_AVERAGE(average_gyr[1], nb_samples_gyr, gyro_dps[1]);
					average_gyr[2] = SLIDING_AVERAGE(average_gyr[2], nb_samples_gyr, gyro_dps[2]);

					snprintf(gyro_str, 60, "Average Gyro:% 8.2f % 8.2f % 8.2f dps", average_gyr[0],
					         average_gyr[1], average_gyr[2]);
				}
			} else if (!stats_en)
				snprintf(gyro_str, 60, "Gyro        :       -        -        -    ");
			else
				snprintf(gyro_str, 60, "Average Gyro:       -        -        -    ");

			if (accel_en || gyro_en) {
				if (!stats_en || ((si_get_time_us() - last_print_time) >= 1 * 1000 * 1000)) {
					nb_samples_acc = 0;
					nb_samples_gyr = 0;
					average_acc[0] = 0;
					average_acc[1] = 0;
					average_acc[2] = 0;
					average_gyr[0] = 0;
					average_gyr[1] = 0;
					average_gyr[2] = 0;
					INV_MSG(INV_MSG_LEVEL_INFO, "%10llu us   %s   %s", timestamp, accel_str,
					        gyro_str);
					last_print_time = si_get_time_us();
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
#if INV_IMU_AUX2_SUPPORTED
	ioc_pad_scenario_aux_ovrd_t ioc_pad_scenario_aux_ovrd = { 0 };
#endif

	rc |= si_board_init();

	/* Configure UART for log */
	rc |= si_config_uart_for_print(SI_UART_ID_FTDI, INV_MSG_LEVEL_DEBUG);

	/* Configure GPIO to call `int_cb` when INT2 fires. */
	rc |= si_init_gpio_int(SI_GPIO_INT2, int_cb);

	/* Init timer peripheral for sleep and get_time */
	rc |= si_init_timers();

#if INV_IMU_AUX2_SUPPORTED
	/* On triple interface chips, need to disable AUX2 to support 4-wire SPI on AUX1
	   This is expected to be done through I2C to not interfere with SPI dedicated to AUX1 */
	rc |= si_io_imu_init(UI_I2C);

	/* Override enable for AUX2_ENABLE with value 0 to disable AUX2 */
	ioc_pad_scenario_aux_ovrd.aux2_enable_ovrd     = 1;
	ioc_pad_scenario_aux_ovrd.aux2_enable_ovrd_val = 0;
	rc |= si_io_imu_write_reg(IOC_PAD_SCENARIO_AUX_OVRD, (uint8_t *)&ioc_pad_scenario_aux_ovrd, 1);
#endif

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

	/* Init transport layer */
	aux_dev.read_reg   = si_io_imu_read_reg;
	aux_dev.write_reg  = si_io_imu_write_reg;
	aux_dev.serif_type = SERIF_TYPE;

	/* Wait 3 ms to ensure device is properly supplied  */
	si_sleep_us(3000);

	/* Check whoami */
	rc |= inv_imu_read_reg(&aux_dev, WHO_AM_I, 1, &whoami);
	SI_CHECK_RC(rc);
	if (whoami != INV_IMU_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Erroneous WHOAMI value.");
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Read 0x%02x", whoami);
		INV_MSG(INV_MSG_LEVEL_ERROR, "  - Expected 0x%02x", INV_IMU_WHOAMI);
		return -1;
	}

	/* Set FSR */
	rc |= inv_imu_set_aux1_accel_fsr(&aux_dev, FS_SEL_AUX_ACCEL_FS_SEL_8_G);
	rc |= inv_imu_set_aux1_gyro_fsr(&aux_dev, FS_SEL_AUX_GYRO_FS_SEL_2000_DPS);
	SI_CHECK_RC(rc);

	/* Set power modes */
	rc |= configure_accel();
	rc |= configure_gyro();

	/*
	 * Configure interrupts pins
	 * - Polarity High
	 * - Pulse mode
	 * - Push-Pull drive
	 */
	int_pin_config.int_polarity = INTX_CONFIG2_INTX_POLARITY_HIGH;
	int_pin_config.int_mode     = INTX_CONFIG2_INTX_MODE_PULSE;
	int_pin_config.int_drive    = INTX_CONFIG2_INTX_DRIVE_PP;
	rc |= inv_imu_set_aux1_pin_config_int(&aux_dev, &int_pin_config);
	SI_CHECK_RC(rc);

	/*
	 * Configure INT2 to trigger on AUX1 DRDY
	 */
	rc |= inv_imu_set_aux1_drdy(&aux_dev, INV_IMU_ENABLE, INV_IMU_INT2);
	SI_CHECK_RC(rc);

	return rc;
}

/* Configure power mode for accel */
static int configure_accel()
{
	int rc = 0;

	if (accel_en) {
		rc |= inv_imu_set_aux1_accel_mode(&aux_dev, INV_IMU_ENABLE);
		/* Discard samples during accel startup time */
		si_sleep_us(ACC_STARTUP_TIME_US);
	} else
		rc |= inv_imu_set_aux1_accel_mode(&aux_dev, INV_IMU_DISABLE);
	SI_CHECK_RC(rc);

	return 0;
}

/* Configure power mode for gyro */
static int configure_gyro()
{
	int rc = 0;

	if (gyro_en) {
		rc |= inv_imu_set_aux1_gyro_mode(&aux_dev, INV_IMU_ENABLE);
		/* Discard samples during gyro startup time */
		si_sleep_us(GYR_STARTUP_TIME_US);
	} else
		rc |= inv_imu_set_aux1_gyro_mode(&aux_dev, INV_IMU_DISABLE);
	SI_CHECK_RC(rc);

	return 0;
}

/* IMU interrupt handler. */
static void int_cb(void *context, unsigned int int_num)
{
	(void)context;

	if (int_num == SI_GPIO_INT2) {
		int2_timestamp = si_get_time_us();
		int2_flag      = 1;
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
	case 'a':
		accel_en = !accel_en;
		rc |= configure_accel();
		rc |= print_current_config();
		break;
	case 'g':
		gyro_en = !gyro_en;
		rc |= configure_gyro();
		rc |= print_current_config();
		break;
	case 's':
		stats_en       = !stats_en;
		nb_samples_acc = 0;
		nb_samples_gyr = 0;
		average_acc[0] = 0;
		average_acc[1] = 0;
		average_acc[2] = 0;
		average_gyr[0] = 0;
		average_gyr[1] = 0;
		average_gyr[2] = 0;
		rc |= print_current_config();
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
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'a' : Toggle accel data");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'g' : Toggle gyro data");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 's' : Print all data or only statistics on a regular basis");
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
	INV_MSG(INV_MSG_LEVEL_INFO, "# Stats: %s", stats_en ? "ON" : "OFF");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	si_sleep_us(2000000); /* Give user some time to read */

	return 0;
}
