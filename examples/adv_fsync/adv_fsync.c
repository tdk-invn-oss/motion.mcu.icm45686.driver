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
 * This example showcases how to enable EIS and get FSYNC tagging information.
 * It enables gyroscope at 200Hz and emulates a camera module FSYNC signal at 30Hz to IMU FSYNC pin.
 * Gyroscope data and FSYNC tag can be either read from FIFO or from IMU Sensor registers, depending on user request
 * through UART.
 */

/*
 * Select communication link between SmartMotion and IMU.
 * SPI4: `UI_SPI4`
 * I2C:  `UI_I2C`
 */
#define SERIF_TYPE UI_SPI4

/* 
 * FSYNC toggle frequency emulating a camera module
 */
#define FSYNC_FREQUENCY_HZ 30

/* Static variables */
static inv_imu_device_t  imu_dev; /* Driver structure */
static volatile int      int1_flag; /* Flag set when INT1 is received */
static volatile uint64_t int1_timestamp; /* Store timestamp when int from IMU fires */
static uint8_t           fifo_data[FIFO_MIRRORING_SIZE]; /* Memory where to store FIFO data */
static uint64_t          timestamp; /* Interrupt timestamp to be used in callback */

/* Static variables for command interface */
static uint8_t use_fifo; /* Indicates if FSYNC is tagged in FIFO or in sensor registers */

/* Static functions definition */
static int  setup_mcu();
static int  setup_imu();
static void int_cb(void *context, unsigned int int_num);
static void sensor_event_cb(inv_imu_sensor_event_t *event);
static void ext_fsync_toggle_cb(void *context);
static int  tag_fsync_in_fifo();
static int  tag_fsync_in_registers();
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
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example FSYNC EIS (using advanced API)");
	INV_MSG(INV_MSG_LEVEL_INFO, "###");

	/* Reset commands interface states */
	use_fifo = 1;

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

			if (use_fifo && int_state.INV_FIFO_THS) {
				rc |= inv_imu_adv_get_data_from_fifo(&imu_dev, fifo_data, &fifo_count);
				SI_CHECK_RC(rc);
				rc |= inv_imu_adv_parse_fifo_data(&imu_dev, fifo_data, fifo_count);
				SI_CHECK_RC(rc);
			} else if (int_state.INV_UI_DRDY) {
				rc |= inv_imu_adv_get_data_from_registers(&imu_dev);
				SI_CHECK_RC(rc);
			}
		}

		rc |= get_uart_command();
	} while (rc == 0);

	return rc;
}

/*
 * Callback called upon timer interrupt, simulate FSYNC signal by toggling GPIO
 */
static void ext_fsync_toggle_cb(void *context)
{
	(void)context;
	si_toggle_gpio_fsync();
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

	/* Send FSYNC signal to IMU to emulate camera module at 30Hz */
	rc |= si_start_gpio_fsync(2 * FSYNC_FREQUENCY_HZ /* FSYNC with rising edge @ 30Hz */,
	                          ext_fsync_toggle_cb);

	return rc;
}

/* Initializes IMU device and apply configuration. */
static int setup_imu()
{
	int                      rc = 0;
	inv_imu_int_pin_config_t int_pin_config;
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

	/* FSYNC configuration */
	rc |= inv_imu_adv_set_int2_pin_usage(&imu_dev, IOC_PAD_SCENARIO_OVRD_INT2_CFG_OVRD_VAL_FSYNC);

	rc |= inv_imu_adv_enable_fsync(&imu_dev);
	SI_CHECK_RC(rc);

	if (use_fifo)
		rc |= tag_fsync_in_fifo();
	else
		rc |= tag_fsync_in_registers();
	SI_CHECK_RC(rc);

	/* Set FSR */
	rc |= inv_imu_set_gyro_fsr(&imu_dev, GYRO_CONFIG0_GYRO_UI_FS_SEL_2000_DPS);
	SI_CHECK_RC(rc);

	/* Set ODR */
	rc |= inv_imu_set_gyro_frequency(&imu_dev, GYRO_CONFIG0_GYRO_ODR_200_HZ);
	SI_CHECK_RC(rc);

	/* Set BW = ODR/4 */
	rc |= inv_imu_set_gyro_ln_bw(&imu_dev, IPREG_SYS1_REG_172_GYRO_UI_LPFBW_DIV_4);
	SI_CHECK_RC(rc);

	/* Enable sensors */
	rc |= inv_imu_adv_enable_gyro_ln(&imu_dev);
	SI_CHECK_RC(rc);

	/* Discard samples during gyro startup time */
	si_sleep_us(GYR_STARTUP_TIME_US);
	if (use_fifo)
		inv_imu_flush_fifo(&imu_dev);

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
	static uint64_t event_fsync_ts        = (uint64_t)(-1);
	static uint16_t event_fsync_delay_cnt = 0;

	if (event->sensor_mask & (1 << INV_SENSOR_FSYNC_EVENT)) {
		event->timestamp_fsync = (uint16_t)((uint64_t)event->timestamp_fsync *
		                                    inv_imu_adv_get_timestamp_resolution_us(&imu_dev));
		event_fsync_ts         = timestamp;
		event_fsync_delay_cnt  = event->timestamp_fsync;
		/* 
		 * Uncomment to print FSYNC events
		 */
		// INV_MSG(INV_MSG_LEVEL_INFO, "%u: FSYNC %hd", (uint32_t)event_fsync_ts, (uint16_t)event_fsync_delay_cnt);
	}

	if (event->sensor_mask & (1 << INV_SENSOR_GYRO)) {
		/*
		 * Gyro data are coded as 16-bits signed (max_lsb = 2^(16-1) = 32768)
		 * with the configured FSR (2000 dps, see `setup_imu()` function).
		 */
		int   max_lsb      = 32768;
		int   gyro_fsr_dps = 2000;
		float gyro_dps[3];
		char  gyro_str[40];

		/* Convert raw data from FIFO data to SI units */
		gyro_dps[0] = (float)(event->gyro[0] * gyro_fsr_dps) / max_lsb;
		gyro_dps[1] = (float)(event->gyro[1] * gyro_fsr_dps) / max_lsb;
		gyro_dps[2] = (float)(event->gyro[2] * gyro_fsr_dps) / max_lsb;

		/* Print data in SI units. */
		snprintf(gyro_str, 40, "Gyro:% 8.2f % 8.2f % 8.2f dps", gyro_dps[0], gyro_dps[1],
		         gyro_dps[2]);

		/* Compute the EIS data */
		if (timestamp == event_fsync_ts) {
			INV_MSG(INV_MSG_LEVEL_INFO, "%10llu us   %s   FSYNC event %hd", timestamp, gyro_str,
			        (uint16_t)event_fsync_delay_cnt);
		} else {
			INV_MSG(INV_MSG_LEVEL_INFO, "%10llu us   %s", timestamp, gyro_str);
		}
	}
}

static int tag_fsync_in_fifo(void)
{
	int                       rc = 0;
	inv_imu_int_state_t       int_config;
	inv_imu_adv_fifo_config_t fifo_config;

	/* Interrupts configuration */
	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
	int_config.INV_FIFO_THS = INV_IMU_ENABLE;
	int_config.INV_UI_FSYNC = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_config);
	SI_CHECK_RC(rc);

	/* FIFO configuration */
	rc |= inv_imu_adv_get_fifo_config(&imu_dev, &fifo_config);
	fifo_config.base_conf.fifo_mode  = FIFO_CONFIG0_FIFO_MODE_SNAPSHOT;
	fifo_config.base_conf.gyro_en    = 1;
	fifo_config.base_conf.accel_en   = 1;
	fifo_config.base_conf.hires_en   = 0;
	fifo_config.base_conf.fifo_wm_th = 1;
	fifo_config.tmst_fsync_en        = INV_IMU_ENABLE;
	fifo_config.fifo_wr_wm_gt_th     = FIFO_CONFIG2_FIFO_WR_WM_EQ_OR_GT_TH;
	rc |= inv_imu_adv_set_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);

	/* Disable FSYNC to tag sensor data in sensor data registers */
	rc |= inv_imu_adv_configure_fsync_ap_tag(&imu_dev, FSYNC_CONFIG0_AP_FSYNC_NO);

	return rc;
}

static int tag_fsync_in_registers(void)
{
	int                       rc = 0;
	inv_imu_int_state_t       int_config;
	inv_imu_adv_fifo_config_t fifo_config;

	/* Interrupts configuration */
	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
	int_config.INV_UI_DRDY  = INV_IMU_ENABLE;
	int_config.INV_UI_FSYNC = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_config);
	SI_CHECK_RC(rc);

	/* FIFO configuration */
	rc |= inv_imu_adv_get_fifo_config(&imu_dev, &fifo_config);
	fifo_config.base_conf.fifo_mode = FIFO_CONFIG0_FIFO_MODE_BYPASS;
	rc |= inv_imu_adv_set_fifo_config(&imu_dev, &fifo_config);
	SI_CHECK_RC(rc);

	/* Configure FSYNC to tag temperature LSB */
	rc |= inv_imu_adv_configure_fsync_ap_tag(&imu_dev, FSYNC_CONFIG0_AP_FSYNC_TEMP);

	return 0;
}

/* Get command from user through UART */
static int get_uart_command()
{
	int  rc  = 0;
	char cmd = 0;

	rc |= si_get_uart_command(SI_UART_ID_FTDI, &cmd);
	SI_CHECK_RC(rc);

	switch (cmd) {
	case 'f':
		use_fifo = 1;
		rc |= tag_fsync_in_fifo();
		rc |= print_current_config();
		rc |= inv_imu_adv_reset_fifo(&imu_dev);
		break;
	case 'r':
		use_fifo = 0;
		rc |= tag_fsync_in_registers();
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
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'f' : Get FSYNC flag and counter from FIFO");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'r' : Get FSYNC flag and counter from IMU Sensor registers");
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
	INV_MSG(INV_MSG_LEVEL_INFO, "# Using %s to get FSYNC flag and counter",
	        use_fifo ? "FIFO" : "IMU Sensor registers");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	si_sleep_us(2000000); /* Give user some time to read */

	return 0;
}