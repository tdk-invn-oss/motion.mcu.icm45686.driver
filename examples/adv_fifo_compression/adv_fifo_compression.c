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
 * This example showcases how to retrieve data from the FIFO while using FIFO compression.
 * It enables accelerometer and/or gyroscope at 50Hz and enables/disable compression upon user request through UART.
 * FIFO compression is enabled by default, with a watermark configuration of 16 samples
 * and a non-compressed frame every 8 frames
 */

/*
 * Select communication link between SmartMotion and IMU.
 * SPI4: `UI_SPI4`
 * I2C:  `UI_I2C`
 */
#define SERIF_TYPE UI_SPI4

#define SLIDING_AVERAGE(AVG, N, DATA) (float)((AVG * (double)(N - 1) + DATA) / (double)N)
#define MAX(a, b)                     ((a) > (b) ? (a) : (b))

/* Contains variables for averaging */
typedef struct {
	uint32_t count;
	float    average[3];
} stats_t;

/* Static variables */
static inv_imu_device_t  imu_dev; /* Driver structure */
static volatile int      int1_flag; /* Flag set when INT1 is received */
static volatile uint64_t int1_timestamp; /* Store timestamp when int from IMU fires */
static uint8_t           fifo_data[FIFO_MIRRORING_SIZE]; /* Memory where to store FIFO data */
static uint64_t          timestamp; /* Interrupt timestamp to be used in callback */
static uint64_t          accel_start_time_us; /* Accel startup time to discard samples */
static uint64_t          gyro_start_time_us; /* Gyro startup time to discard samples */
static stats_t           acc_stats; /* Stats for accel */
static stats_t           gyr_stats; /* Stats for gyro */
static stats_t           temp_stats; /* Stats for temp */

/* Static variables for command interface */
static uint8_t accel_en; /* Indicates accel state */
static uint8_t gyro_en; /* Indicates gyro state */
static uint8_t compression_en; /* Indicates FIFO compression state */

/* Static functions definition */
static int  setup_mcu();
static int  setup_imu();
static int  configure_fifo();
static int  configure_accel();
static int  configure_gyro();
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
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example FIFO Compression (using advanced API)");
	INV_MSG(INV_MSG_LEVEL_INFO, "###");

	/* Reset commands interface states */
	accel_en       = 1;
	gyro_en        = 1;
	compression_en = 1;

	rc |= setup_imu();
	SI_CHECK_RC(rc);

	rc |= configure_accel();
	SI_CHECK_RC(rc);

	rc |= configure_gyro();
	SI_CHECK_RC(rc);

	/* Reset timestamp and interrupt flag */
	int1_flag      = 0;
	int1_timestamp = 0;

	do {
		/* Poll device for data */
		if (int1_flag) {
			inv_imu_int_state_t int_state;
			char                accel_str[60];
			char                gyro_str[60];
			char                temp_str[40];
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
				/* Reset stats structures */
				memset(&acc_stats, 0, sizeof(acc_stats));
				memset(&gyr_stats, 0, sizeof(gyr_stats));
				memset(&temp_stats, 0, sizeof(temp_stats));

				rc |= inv_imu_adv_get_data_from_fifo(&imu_dev, fifo_data, &fifo_count);
				SI_CHECK_RC(rc);
				rc |= inv_imu_adv_parse_fifo_data(&imu_dev, fifo_data, fifo_count);
				SI_CHECK_RC(rc);

				INV_MSG(INV_MSG_LEVEL_INFO, "SI %10llu us   FIFO count %u   Sample count %u",
				        timestamp, (uint32_t)fifo_count, MAX(acc_stats.count, gyr_stats.count));

				if (accel_en)
					snprintf(accel_str, 60, "Average Accel:% 8.2f % 8.2f % 8.2f g",
					         acc_stats.average[0], acc_stats.average[1], acc_stats.average[2]);
				else
					snprintf(accel_str, 60, "Average Accel:       -        -        -  ");

				if (gyro_en)
					snprintf(gyro_str, 60, "Average Gyro:% 8.2f % 8.2f % 8.2f dps",
					         gyr_stats.average[0], gyr_stats.average[1], gyr_stats.average[2]);
				else
					snprintf(gyro_str, 60, "Average Gyro:       -        -        -    ");

				/*
				 * In compressed frames, temperature is only available if
				 * accel and gyro are enabled.
				 */
				if (compression_en && !(accel_en && gyro_en))
					snprintf(temp_str, 40, "Average Temp: -");
				else
					snprintf(temp_str, 40, "Average Temp: % 4.2f degC", temp_stats.average[0]);

				INV_MSG(INV_MSG_LEVEL_INFO, "SI %10llu us   %s   %s   %s", timestamp, accel_str,
				        gyro_str, temp_str);
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
	int                      rc = 0;
	inv_imu_int_pin_config_t int_pin_config;
	inv_imu_int_state_t      int_config;
	inv_imu_adv_var_t *      e = (inv_imu_adv_var_t *)imu_dev.adv_var;

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
	rc |= configure_fifo();
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

	return rc;
}

static int configure_fifo(void)
{
	int                       rc = 0;
	inv_imu_adv_fifo_config_t fifo_config;

	/* It is recommended to empty the FIFO before enabling compression */
	rc |= inv_imu_adv_reset_fifo(&imu_dev);
	SI_CHECK_RC(rc);

	/* FIFO configuration */
	rc |= inv_imu_adv_get_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);
	fifo_config.base_conf.gyro_en   = gyro_en;
	fifo_config.base_conf.accel_en  = accel_en;
	fifo_config.base_conf.fifo_mode = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT;
	fifo_config.tmst_fsync_en       = INV_IMU_ENABLE;
	fifo_config.fifo_wr_wm_gt_th    = FIFO_CONFIG2_FIFO_WR_WM_EQ_OR_GT_TH;
	/*
	 * Enable FIFO compression
	 * Non-compressed frame every 8 frames
	 */
	fifo_config.base_conf.fifo_wm_th = 16; // watermark set to 16 frames = 256 bytes
	fifo_config.base_conf.fifo_depth = FIFO_CONFIG0_FIFO_DEPTH_APEX;
	fifo_config.comp_nc_flow_cfg     = FIFO_CONFIG4_FIFO_COMP_NC_FLOW_CFG_EVERY_8_FR;
	fifo_config.comp_en              = compression_en;
	rc |= inv_imu_adv_set_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);

	return rc;
}
/* Configure FIFO and power mode for accel */
static int configure_accel()
{
	int                       rc = 0;
	inv_imu_adv_fifo_config_t fifo_config;

	rc |= inv_imu_adv_get_fifo_config(&imu_dev, &fifo_config);
	fifo_config.base_conf.accel_en = accel_en;
	rc |= inv_imu_adv_set_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);

	if (accel_en) {
		rc |= inv_imu_adv_enable_accel_ln(&imu_dev);
		accel_start_time_us = si_get_time_us();
	} else {
		rc |= inv_imu_adv_disable_accel(&imu_dev);
		memset(&acc_stats, 0, sizeof(acc_stats));
		if (compression_en || !gyro_en)
			memset(&temp_stats, 0, sizeof(temp_stats));
	}
	SI_CHECK_RC(rc);

	return rc;
}

/* Configure FIFO and power mode for gyro */
static int configure_gyro()
{
	int                       rc = 0;
	inv_imu_adv_fifo_config_t fifo_config;

	rc |= inv_imu_adv_get_fifo_config(&imu_dev, &fifo_config);
	fifo_config.base_conf.gyro_en = gyro_en;
	rc |= inv_imu_adv_set_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);

	if (gyro_en) {
		rc |= inv_imu_adv_enable_gyro_ln(&imu_dev);
		gyro_start_time_us = si_get_time_us();
	} else {
		rc |= inv_imu_adv_disable_gyro(&imu_dev);
		memset(&gyr_stats, 0, sizeof(gyr_stats));
		if (compression_en || !accel_en)
			memset(&temp_stats, 0, sizeof(temp_stats));
	}
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
	/*
	 * Accel and gyro data are coded as 16-bits signed (max_lsb = 2^(16-1) = 32768)
	 * with the configured FSR (4 g and 2000 dps, see `setup_imu()` function). Temperature is coded
	 * as 8-bits signed with a scale factor of 2 and an offset of 25 °C.
	 */
	int max_lsb = 32768;

	/* Discard samples until startup time has elapsed. */
	if ((event->sensor_mask & (1 << INV_SENSOR_ACCEL)) && (accel_start_time_us != UINT32_MAX)) {
		if ((si_get_time_us() - accel_start_time_us) >= ACC_STARTUP_TIME_US)
			accel_start_time_us = UINT32_MAX; /* Indicates startup time has elapsed */
		else
			event->sensor_mask &= ~(1 << INV_SENSOR_ACCEL); /* Mark data as invalid */
	}
	if ((event->sensor_mask & (1 << INV_SENSOR_GYRO)) && (gyro_start_time_us != UINT32_MAX)) {
		if ((si_get_time_us() - gyro_start_time_us) >= GYR_STARTUP_TIME_US)
			gyro_start_time_us = UINT32_MAX; /* Indicates startup time has elapsed */
		else
			event->sensor_mask &= ~(1 << INV_SENSOR_GYRO); /* Mark data as invalid */
	}

	/* Convert data to SI units */
	if (event->sensor_mask & (1 << INV_SENSOR_ACCEL)) {
		float accel_g[3];
		int   accel_fsr_g = 4;
		acc_stats.count++;
		accel_g[0]           = (float)(event->accel[0] * accel_fsr_g) / max_lsb;
		accel_g[1]           = (float)(event->accel[1] * accel_fsr_g) / max_lsb;
		accel_g[2]           = (float)(event->accel[2] * accel_fsr_g) / max_lsb;
		acc_stats.average[0] = SLIDING_AVERAGE(acc_stats.average[0], acc_stats.count, accel_g[0]);
		acc_stats.average[1] = SLIDING_AVERAGE(acc_stats.average[1], acc_stats.count, accel_g[1]);
		acc_stats.average[2] = SLIDING_AVERAGE(acc_stats.average[2], acc_stats.count, accel_g[2]);
	}

	if (event->sensor_mask & (1 << INV_SENSOR_GYRO)) {
		float gyro_dps[3];
		int   gyro_fsr_dps = 2000;
		gyr_stats.count++;
		gyro_dps[0]          = (float)(event->gyro[0] * gyro_fsr_dps) / max_lsb;
		gyro_dps[1]          = (float)(event->gyro[1] * gyro_fsr_dps) / max_lsb;
		gyro_dps[2]          = (float)(event->gyro[2] * gyro_fsr_dps) / max_lsb;
		gyr_stats.average[0] = SLIDING_AVERAGE(gyr_stats.average[0], gyr_stats.count, gyro_dps[0]);
		gyr_stats.average[1] = SLIDING_AVERAGE(gyr_stats.average[1], gyr_stats.count, gyro_dps[1]);
		gyr_stats.average[2] = SLIDING_AVERAGE(gyr_stats.average[2], gyr_stats.count, gyro_dps[2]);
	}

	if (event->sensor_mask & (1 << INV_SENSOR_TEMPERATURE)) {
		int   temp_scale_factor = 2;
		int   temp_offset_c     = 25;
		float temp_degc = (float)temp_offset_c + ((float)event->temperature / temp_scale_factor);
		temp_stats.count++;
		temp_stats.average[0] = SLIDING_AVERAGE(temp_stats.average[0], temp_stats.count, temp_degc);
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
		INV_MSG(INV_MSG_LEVEL_INFO, "%s Accel.", accel_en ? "Enabling" : "Disabling");
		break;
	case 'g':
		gyro_en = !gyro_en;
		rc |= configure_gyro();
		INV_MSG(INV_MSG_LEVEL_INFO, "%s Gyro.", gyro_en ? "Enabling" : "Disabling");
		break;
	case 'f':
		compression_en = !compression_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s FIFO Compression.",
		        compression_en ? "Enabling" : "Disabling");
		rc |= configure_fifo();
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
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'f' : Toggle FIFO compression");
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
	INV_MSG(INV_MSG_LEVEL_INFO, "# FIFO compression: %s", compression_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	si_sleep_us(2000000); /* Give user some time to read */

	return 0;
}