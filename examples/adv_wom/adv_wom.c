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
#include "imu/inv_imu_driver_advanced.h"

/* Board drivers */
#include "system_interface.h"

/* std */
#include <stdio.h>

/*
 * This example showcases how to configure and use WOM.
 * It starts accelerometer in LP at 12.5Hz and will notify a WOM when any of the 3 axis exceeds the threshold.
 * The threshold can be configured through UART to be either 200 mg or 24 mg and defaults to 200 mg
 */

/*
 * Select communication link between SmartMotion and IMU.
 * SPI4: `UI_SPI4`
 * I2C:  `UI_I2C`
 */
#define SERIF_TYPE UI_SPI4

/* Initial WOM threshold to be applied to IMU in mg */
#define WOM_THRESHOLD_INITIAL_MG 200
/* WOM threshold to be applied to IMU in mg if low threshold is requested*/
#define WOM_THRESHOLD_LOW_MG 24

/* WOM threshold to be applied to IMU, ranges from 1 to 255, in 4 mg unit */
static uint8_t wom_threshold_high = WOM_THRESHOLD_INITIAL_MG / 4;
static uint8_t wom_threshold_low  = WOM_THRESHOLD_LOW_MG / 4;

/* Static variables */
static inv_imu_device_t  imu_dev; /* Driver structure */
static volatile int      int1_flag; /* Flag set when INT1 is received */
static volatile uint64_t int1_timestamp; /* Store timestamp when int from IMU fires */

/* Static variables for command interface */
static uint8_t wom_high_thr_en; /* Indicates WOM state */

/* Static functions definition */
static int  setup_mcu();
static int  setup_imu();
static int  configure_and_enable_wom();
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
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example WOM");
	INV_MSG(INV_MSG_LEVEL_INFO, "###");

	/* Reset commands interface states */
	wom_high_thr_en = 1;

	rc |= setup_imu();
	SI_CHECK_RC(rc);

	rc |= print_current_config();

	/* Reset timestamp and interrupt flag */
	int1_flag      = 0;
	int1_timestamp = 0;

	do {
		/* Poll device for data */
		if (int1_flag) {
			uint64_t            timestamp;
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

			/* If WOM status is set */
			if (int_state.INV_WOM_X || int_state.INV_WOM_Y || int_state.INV_WOM_Z) {
				INV_MSG(INV_MSG_LEVEL_INFO, "WoM at %10llu (X, Y, Z): %hhu, %hhu, %hhu", timestamp,
				        int_state.INV_WOM_X, int_state.INV_WOM_Y, int_state.INV_WOM_Z);
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

	rc |= configure_and_enable_wom();
	SI_CHECK_RC(rc);

	return rc;
}

static int configure_and_enable_wom(void)
{
	int                 rc = 0;
	inv_imu_int_state_t int_config;

	/* Interrupts configuration: Enable only WOM interrupts */
	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
	int_config.INV_WOM_X = INV_IMU_ENABLE;
	int_config.INV_WOM_Y = INV_IMU_ENABLE;
	int_config.INV_WOM_Z = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_config);
	SI_CHECK_RC(rc);

	/* Configure WOM to produce signal when at least one axis exceed wom_threshold_high/low */
	rc |= inv_imu_adv_configure_wom(&imu_dev,
	                                wom_high_thr_en ? wom_threshold_high : wom_threshold_low,
	                                wom_high_thr_en ? wom_threshold_high : wom_threshold_low,
	                                wom_high_thr_en ? wom_threshold_high : wom_threshold_low,
	                                TMST_WOM_CONFIG_WOM_INT_MODE_ORED,
	                                TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL);
	rc |= inv_imu_adv_enable_wom(&imu_dev);
	SI_CHECK_RC(rc);

	rc |= inv_imu_set_accel_frequency(&imu_dev, ACCEL_CONFIG0_ACCEL_ODR_12_5_HZ);
	SI_CHECK_RC(rc);

	/* Select WUOSC clock to have accel in ULP (lowest power mode) */
	rc |= inv_imu_select_accel_lp_clk(&imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_WUOSC);
	SI_CHECK_RC(rc);

	/* Set 1x averaging, in order to minimize power consumption */
	rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_1);
	SI_CHECK_RC(rc);

	rc |= inv_imu_adv_enable_accel_lp(&imu_dev);
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

/* Get command from user through UART */
static int get_uart_command()
{
	int  rc  = 0;
	char cmd = 0;

	rc |= si_get_uart_command(SI_UART_ID_FTDI, &cmd);
	SI_CHECK_RC(rc);

	switch (cmd) {
	case 'w':
		wom_high_thr_en = !wom_high_thr_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "WOM threshold %s.", wom_high_thr_en ? "High" : "Low");
		rc |= configure_and_enable_wom();
		SI_CHECK_RC(rc);
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
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'w' : Toggle WOM threshold high (200mg) / low (24mg)");
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
	INV_MSG(INV_MSG_LEVEL_INFO, "# WOM threshold: %s", wom_high_thr_en ? "High" : "Low");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	si_sleep_us(2000000); /* Give user some time to read */

	return 0;
}
