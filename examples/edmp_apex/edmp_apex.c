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
#include "imu/inv_imu_edmp.h"
#include "imu/inv_imu_driver_advanced.h"

/* Board drivers */
#include "system_interface.h"

/* std */
#include <stdio.h>

/*
 * This example showcases how to configure and use APEX features
 * It allows enabling individually pedometer, SMD, tilt, raise-to-wake,
 * tap and freefall eDMP features.
 * It automatically enables accelerometer at appropriate frequency and in optimal power mode depending
 * on requested feature combination.
 */

/*
 * Select communication link between SmartMotion and IMU.
 * SPI4: `UI_SPI4`
 * I2C:  `UI_I2C`
 */
#define SERIF_TYPE UI_SPI4

/*
 * WOM threshold value in mg.
 * 1g/256 resolution (wom_th = mg * 256 / 1000)
 */
#define DEFAULT_WOM_THS_MG 52 >> 2 // 52 mg

/* Static variables */
static inv_imu_device_t  imu_dev; /* Driver structure */
static volatile int      int1_flag; /* Flag set when INT1 is received */
static volatile uint64_t int1_timestamp; /* Store timestamp when int from IMU fires */
static uint32_t          odr_us; /* Store current ODR for accel/EDMP */

/* Static variables for command interface */
static uint8_t pedometer_en; /* Indicates pedometer state */
static uint8_t smd_en; /* Indicates SMD state */
static uint8_t tilt_en; /* Indicates tilt state */
static uint8_t r2w_en; /* Indicates R2W state */
static uint8_t tap_en; /* Indicates tap state */
static uint8_t ff_en; /* Indicates freefall state */
static uint8_t lowg_en; /* Indicates lowg state */
static uint8_t highg_en; /* Indicates highg state */
static uint8_t power_save_en; /* Indicates power save mode state */

/* Static functions definition */
static int  setup_mcu();
static int  setup_imu();
static int  configure_and_enable_edmp_algo();
static void int_cb(void *context, unsigned int int_num);
static char convert_tap_axis_to_str(inv_imu_edmp_tap_axis_t axis);
static char convert_tap_dir_to_str(inv_imu_edmp_tap_dir_t dir);
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
	INV_MSG(INV_MSG_LEVEL_INFO, "### Example EDMP APEX");
	INV_MSG(INV_MSG_LEVEL_INFO, "###");

	/* Reset commands interface states */
	pedometer_en  = 1;
	smd_en        = 1;
	tilt_en       = 1;
	r2w_en        = 1;
	tap_en        = 1;
	ff_en         = 1;
	lowg_en       = 1;
	highg_en      = 1;
	power_save_en = 0;

	rc |= setup_imu();
	SI_CHECK_RC(rc);

	rc |= print_current_config();

	/* Reset timestamp and interrupt flag */
	int1_flag      = 0;
	int1_timestamp = 0;

	do {
		/* Poll device for data */
		if (int1_flag) {
			static uint32_t          step_cnt_ovflw = 0;
			uint64_t                 timestamp;
			inv_imu_int_state_t      int_state;
			inv_imu_edmp_int_state_t apex_state = { 0 };

			si_disable_irq();
			/* Clear interrupt flag */
			int1_flag = 0;
			/* Retrieve timestamp */
			timestamp = int1_timestamp;
			si_enable_irq();

			/* Read interrupt status */
			rc |= inv_imu_get_int_status(&imu_dev, INV_IMU_INT1, &int_state);
			SI_CHECK_RC(rc);

			/* If APEX status is set */
			if (int_state.INV_EDMP_EVENT) {
				/* Read APEX interrupt status */
				rc |= inv_imu_edmp_get_int_apex_status(&imu_dev, &apex_state);
				SI_CHECK_RC(rc);

				/* Pedometer */
				if (pedometer_en && apex_state.INV_STEP_CNT_OVFL) {
					step_cnt_ovflw++;
					INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   STEP_CNT_OVFL", timestamp);
				}
				if (pedometer_en && apex_state.INV_STEP_DET) {
					inv_imu_edmp_pedometer_data_t ped_data;
					char                          step_count_str[100];
					/* If running faster than 50 Hz, pedometer is decimated to 50 Hz */
					uint32_t ped_odr_us = odr_us > 20000 ? odr_us : 20000;

					rc |= inv_imu_edmp_get_pedometer_data(&imu_dev, &ped_data);
					if (rc == INV_IMU_OK) {
						snprintf(step_count_str, 100,
						         "count: %5lu steps   cadence: %.1f steps/s   activity: %s",
						         (unsigned long)ped_data.step_cnt + (step_cnt_ovflw * UINT16_MAX),
						         1000000 / ((float)ped_data.step_cadence * 0.25 * ped_odr_us),
						         ped_data.activity_class == INV_IMU_EDMP_RUN ?
                                     "Run" :
                                     (ped_data.activity_class == INV_IMU_EDMP_WALK ?
                                          "Walk" :
                                          /* INV_IMU_EDMP_UNKOWN */ "Unknown"));
					}
					SI_CHECK_RC(rc);

					INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   STEP_DET     %s", timestamp,
					        step_count_str);
				}

				/* SMD */
				if (smd_en && apex_state.INV_SMD)
					INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   SMD", timestamp);

				/* Tilt */
				if (tilt_en && apex_state.INV_TILT_DET)
					INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   TILT", timestamp);

				/* R2W */
				if (r2w_en && apex_state.INV_R2W)
					INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   R2W_WAKE", timestamp);
				if (r2w_en && apex_state.INV_R2W_SLEEP)
					INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   R2W_SLEEP", timestamp);

				/* Tap */
				if (tap_en && apex_state.INV_TAP) {
					inv_imu_edmp_tap_data_t tap_data;
					char                    tap_dir_str[20];
					char                    tap_str[80];

					rc |= inv_imu_edmp_get_tap_data(&imu_dev, &tap_data);
					SI_CHECK_RC(rc);

					snprintf(tap_dir_str, 20, "Direction: %c%c",
					         convert_tap_axis_to_str(tap_data.axis),
					         convert_tap_dir_to_str(tap_data.direction));

					if (tap_data.num == INV_IMU_EDMP_TAP_DOUBLE)
						snprintf(tap_str, 80, "%s   %s   duration: %lu us", "Double", tap_dir_str,
						         (unsigned long)tap_data.double_tap_timing * odr_us);
					else
						snprintf(tap_str, 80, "%s   %s", "Single", tap_dir_str);

					INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   TAP     %s", timestamp, tap_str);
				}

				/* FreeFall */
				if (ff_en && apex_state.INV_FF) {
					uint16_t duration;

					rc |= inv_imu_edmp_get_ff_data(&imu_dev, &duration);
					SI_CHECK_RC(rc);

					INV_MSG(INV_MSG_LEVEL_INFO,
					        "   %10llu us   FREEFALL     duration: %u samples (%u ms)", timestamp,
					        duration, (duration * odr_us) / 1000);
				}

				/* LowG */
				if (lowg_en && apex_state.INV_LOWG)
					INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   LOW_G", timestamp);

				/* HighG */
				if (highg_en && apex_state.INV_HIGHG)
					INV_MSG(INV_MSG_LEVEL_INFO, "   %10llu us   HIGH_G", timestamp);
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

	/* Interrupts configuration: Enable only EDMP interrupt */
	memset(&int_config, INV_IMU_DISABLE, sizeof(int_config));
	int_config.INV_EDMP_EVENT = INV_IMU_ENABLE;
	rc |= inv_imu_set_config_int(&imu_dev, INV_IMU_INT1, &int_config);
	SI_CHECK_RC(rc);

	/* Initialize APEX */
	rc |= inv_imu_edmp_init_apex(&imu_dev);
	rc |= configure_and_enable_edmp_algo();
	SI_CHECK_RC(rc);

	return rc;
}

static int configure_and_enable_edmp_algo()
{
	int                            rc = 0;
	dmp_ext_sen_odr_cfg_apex_odr_t dmp_odr;
	accel_config0_accel_odr_t      accel_odr;
	inv_imu_edmp_apex_parameters_t apex_parameters;
	inv_imu_edmp_int_state_t       apex_int_config;

	/* Configure ODR depending on which feature is enabled */
	if (ff_en || lowg_en || highg_en) {
		/* 800 Hz */
		odr_us    = 1250;
		dmp_odr   = DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ;
		accel_odr = ACCEL_CONFIG0_ACCEL_ODR_800_HZ;
	} else if (tap_en) {
		/* 400 Hz */
		odr_us    = 2500;
		dmp_odr   = DMP_EXT_SEN_ODR_CFG_APEX_ODR_400_HZ;
		accel_odr = ACCEL_CONFIG0_ACCEL_ODR_400_HZ;
	} else if (r2w_en) {
		/* 100 Hz */
		odr_us    = 10000;
		dmp_odr   = DMP_EXT_SEN_ODR_CFG_APEX_ODR_100_HZ;
		accel_odr = ACCEL_CONFIG0_ACCEL_ODR_100_HZ;
	} else {
		/* 50 Hz */
		odr_us    = 20000;
		dmp_odr   = DMP_EXT_SEN_ODR_CFG_APEX_ODR_50_HZ;
		accel_odr = ACCEL_CONFIG0_ACCEL_ODR_50_HZ;
	}

	/* Set EDMP ODR */
	rc |= inv_imu_edmp_set_frequency(&imu_dev, dmp_odr);
	SI_CHECK_RC(rc);

	/* Set ODR */
	rc |= inv_imu_set_accel_frequency(&imu_dev, accel_odr);
	SI_CHECK_RC(rc);

	/* Set BW = ODR/4 */
	rc |= inv_imu_set_accel_ln_bw(&imu_dev, IPREG_SYS2_REG_131_ACCEL_UI_LPFBW_DIV_4);
	SI_CHECK_RC(rc);

	/* Select WUOSC clock to have accel in ULP (lowest power mode) */
	rc |= inv_imu_select_accel_lp_clk(&imu_dev, SMC_CONTROL_0_ACCEL_LP_CLK_WUOSC);
	SI_CHECK_RC(rc);

	/* Set AVG to 1x */
	rc |= inv_imu_set_accel_lp_avg(&imu_dev, IPREG_SYS2_REG_129_ACCEL_LP_AVG_1);
	SI_CHECK_RC(rc);

	/* Ensure all DMP features are disabled before running init procedure */
#if INV_IMU_USE_BASIC_SMD
	/* Disable WOM required for Basic SMD */
	rc |= inv_imu_adv_disable_wom(&imu_dev);
#endif
	rc |= inv_imu_edmp_disable_pedometer(&imu_dev);
	rc |= inv_imu_edmp_disable_smd(&imu_dev);
	rc |= inv_imu_edmp_disable_tilt(&imu_dev);
	rc |= inv_imu_edmp_disable_r2w(&imu_dev);
	rc |= inv_imu_edmp_disable_tap(&imu_dev);
	rc |= inv_imu_edmp_disable_ff(&imu_dev);
	rc |= inv_imu_edmp_disable(&imu_dev);
	SI_CHECK_RC(rc);

	/* Request DMP to re-initialize APEX */
	rc |= inv_imu_edmp_recompute_apex_decimation(&imu_dev);
	SI_CHECK_RC(rc);

	/* Configure APEX parameters */
	rc |= inv_imu_edmp_get_apex_parameters(&imu_dev, &apex_parameters);
	apex_parameters.r2w_sleep_time_out = 6400; /* 6.4 s */
	if (tap_en) {
		/* TAP supports 400 Hz and 800 Hz ODR */
		if (dmp_odr == DMP_EXT_SEN_ODR_CFG_APEX_ODR_800_HZ) {
			apex_parameters.tap_tmax             = TAP_TMAX_800HZ;
			apex_parameters.tap_tmin             = TAP_TMIN_800HZ;
			apex_parameters.tap_smudge_reject_th = TAP_SMUDGE_REJECT_THR_800HZ;
		} else {
			apex_parameters.tap_tmax             = TAP_TMAX_400HZ;
			apex_parameters.tap_tmin             = TAP_TMIN_400HZ;
			apex_parameters.tap_smudge_reject_th = TAP_SMUDGE_REJECT_THR_400HZ;
		}
	}
	apex_parameters.power_save_en = power_save_en;
	if (power_save_en) {
		rc |= inv_imu_adv_configure_wom(&imu_dev, DEFAULT_WOM_THS_MG, DEFAULT_WOM_THS_MG,
		                                DEFAULT_WOM_THS_MG, TMST_WOM_CONFIG_WOM_INT_MODE_ANDED,
		                                TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL);
		rc |= inv_imu_adv_enable_wom(&imu_dev);
	} else {
		rc |= inv_imu_adv_disable_wom(&imu_dev);
	}
	rc |= inv_imu_edmp_set_apex_parameters(&imu_dev, &apex_parameters);
	SI_CHECK_RC(rc);

	/* Set accel in low-power mode if ODR slower than 800 Hz, otherwise in low-noise mode */
	if (odr_us <= 1250 /* 800 Hz and faster */)
		rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LN);
	else
		rc |= inv_imu_set_accel_mode(&imu_dev, PWR_MGMT0_ACCEL_MODE_LP);
	SI_CHECK_RC(rc);

	/* Wait for accel startup time */
	si_sleep_us(ACC_STARTUP_TIME_US);

	/* Disable all APEX interrupt and enable only the one we need */
	memset(&apex_int_config, INV_IMU_DISABLE, sizeof(apex_int_config));

	/* Enable requested features */
	if (pedometer_en) {
		rc |= inv_imu_edmp_enable_pedometer(&imu_dev);
		apex_int_config.INV_STEP_CNT_OVFL = INV_IMU_ENABLE;
		apex_int_config.INV_STEP_DET      = INV_IMU_ENABLE;
	}

	if (smd_en) {
#if INV_IMU_USE_BASIC_SMD
		/* Configure and enable WOM required for Basic SMD */
		rc |= inv_imu_adv_configure_wom(&imu_dev, 6, 6, 6, // 6 * 1/256mg ~= 23.44mg for each axis
		                                TMST_WOM_CONFIG_WOM_INT_MODE_ORED,
		                                TMST_WOM_CONFIG_WOM_INT_DUR_1_SMPL);
		rc |= inv_imu_adv_enable_wom(&imu_dev);
#endif
		rc |= inv_imu_edmp_enable_smd(&imu_dev);
		apex_int_config.INV_SMD = INV_IMU_ENABLE;
	}

	if (tilt_en) {
		rc |= inv_imu_edmp_enable_tilt(&imu_dev);
		apex_int_config.INV_TILT_DET = INV_IMU_ENABLE;
	}

	if (r2w_en) {
		rc |= inv_imu_edmp_enable_r2w(&imu_dev);
		apex_int_config.INV_R2W       = INV_IMU_ENABLE;
		apex_int_config.INV_R2W_SLEEP = INV_IMU_ENABLE;
	}

	if (tap_en) {
		rc |= inv_imu_edmp_enable_tap(&imu_dev);
		apex_int_config.INV_TAP = INV_IMU_ENABLE;
	}

	if (ff_en) {
		rc |= inv_imu_edmp_enable_ff(&imu_dev);
		apex_int_config.INV_FF = INV_IMU_ENABLE;
	}

	if (lowg_en) {
		rc |= inv_imu_edmp_enable_ff(&imu_dev);
		apex_int_config.INV_LOWG = INV_IMU_ENABLE;
	}

	if (highg_en) {
		rc |= inv_imu_edmp_enable_ff(&imu_dev);
		apex_int_config.INV_HIGHG = INV_IMU_ENABLE;
	}

	/* Apply interrupt configuration */
	rc |= inv_imu_edmp_set_config_int_apex(&imu_dev, &apex_int_config);
	SI_CHECK_RC(rc);

	/* Enable EDMP if at least one feature is enabled */
	if (pedometer_en || smd_en || tilt_en || r2w_en || tap_en || ff_en || lowg_en || highg_en)
		rc |= inv_imu_edmp_enable(&imu_dev);

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

static char convert_tap_axis_to_str(inv_imu_edmp_tap_axis_t axis)
{
	if (axis == INV_IMU_EDMP_TAP_AXIS_X)
		return 'X';
	else if (axis == INV_IMU_EDMP_TAP_AXIS_Y)
		return 'Y';
	else
		return 'Z';
}

static char convert_tap_dir_to_str(inv_imu_edmp_tap_dir_t dir)
{
	if (dir == INV_IMU_EDMP_TAP_DIR_POSITIVE)
		return '+';
	else
		return '-';
}

/* Get command from user through UART */
static int get_uart_command()
{
	int  rc  = 0;
	char cmd = 0;

	rc |= si_get_uart_command(SI_UART_ID_FTDI, &cmd);
	SI_CHECK_RC(rc);

	switch (cmd) {
	case 'p':
		pedometer_en = !pedometer_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s Pedometer.", pedometer_en ? "Enabling" : "Disabling");
		rc |= configure_and_enable_edmp_algo();
		SI_CHECK_RC(rc);
		break;
	case 's':
		smd_en = !smd_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s SMD.", smd_en ? "Enabling" : "Disabling");
		rc |= configure_and_enable_edmp_algo();
		SI_CHECK_RC(rc);
		break;
	case 't':
		tilt_en = !tilt_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s Tilt.", tilt_en ? "Enabling" : "Disabling");
		rc |= configure_and_enable_edmp_algo();
		SI_CHECK_RC(rc);
		break;
	case 'r':
		r2w_en = !r2w_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s R2W.", r2w_en ? "Enabling" : "Disabling");
		rc |= configure_and_enable_edmp_algo();
		SI_CHECK_RC(rc);
		break;
	case 'a':
		tap_en = !tap_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s Tap.", tap_en ? "Enabling" : "Disabling");
		rc |= configure_and_enable_edmp_algo();
		SI_CHECK_RC(rc);
		break;
	case 'f':
		ff_en = !ff_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s Freefall.", ff_en ? "Enabling" : "Disabling");
		rc |= configure_and_enable_edmp_algo();
		SI_CHECK_RC(rc);
		break;
	case 'l':
		lowg_en = !lowg_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s LowG.", lowg_en ? "Enabling" : "Disabling");
		rc |= configure_and_enable_edmp_algo();
		SI_CHECK_RC(rc);
		break;
	case 'i':
		highg_en = !highg_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s HighG.", highg_en ? "Enabling" : "Disabling");
		rc |= configure_and_enable_edmp_algo();
		SI_CHECK_RC(rc);
		break;
	case 'c':
		rc |= print_current_config();
		break;
	case 'o':
		power_save_en = !power_save_en;
		INV_MSG(INV_MSG_LEVEL_INFO, "%s Power Save mode.",
		        power_save_en ? "Enabling" : "Disabling");
		rc |= configure_and_enable_edmp_algo();
		SI_CHECK_RC(rc);
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
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'p' : Toggle Pedometer");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 's' : Toggle SMD");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 't' : Toggle Tilt");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'r' : Toggle R2W");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'a' : Toggle Tap");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'f' : Toggle Freefall");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'l' : Toggle LowG");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'i' : Toggle HighG");
	INV_MSG(INV_MSG_LEVEL_INFO, "# 'o' : Toggle Power Save mode");
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
	INV_MSG(INV_MSG_LEVEL_INFO, "# Pedometer: %s", pedometer_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "# SMD: %s", smd_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Tilt: %s", tilt_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "# R2W: %s", r2w_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Tap: %s", tap_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Freefall: %s", ff_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "# LowG: %s", lowg_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "# HighG: %s", highg_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "# Power Save mode: %s", power_save_en ? "Enabled" : "Disabled");
	INV_MSG(INV_MSG_LEVEL_INFO, "#");
	INV_MSG(INV_MSG_LEVEL_INFO, "# ODR: %u Hz", (uint32_t)((float)1000000 / odr_us));
	INV_MSG(INV_MSG_LEVEL_INFO, "#");

	si_sleep_us(2000000); /* Give user some time to read */

	return 0;
}
