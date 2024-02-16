/*
 * ________________________________________________________________________________________________________
 * Copyright © 2021 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 *
 * ________________________________________________________________________________________________________
 */

#ifndef __INV_IMU_EDMP_DEFS_H__
#define __INV_IMU_EDMP_DEFS_H__

#ifdef __cplusplus
extern "C" {
#endif

#define EDMP_RAM_BASE	0x0
#define EDMP_ROM_BASE	0x4000
#define EDMP_ROM_DATA_SIZE	0x4C0
#define APEX_FEATURE_STACK_END	0x500
#define EDMP_RAM_FEATURE_PRGM_RAM_BASE	0x500
#define EDMP_HOST_INT_TAP_DET_POS	0x0
#define EDMP_HOST_INT_HIGHG_DET_POS	0x1
#define EDMP_HOST_INT_LOWG_DET_POS	0x2
#define EDMP_HOST_INT_TILT_DET_POS	0x3
#define EDMP_HOST_INT_STEP_CNT_OVFL_DET_POS	0x4
#define EDMP_HOST_INT_STEP_DET_POS	0x5
#define EDMP_HOST_INT_FF_DET_POS	0x6
#define EDMP_HOST_INT_R2W_WAKE_DET_POS	0x7
#define EDMP_HOST_INT_B2S_DET_POS	0x7
#define EDMP_HOST_INT_REVB2S_DET_POS	0x0
#define EDMP_HOST_INT_R2W_SLEEP_DET_POS	0x0
#define EDMP_HOST_INT_SMD_DET_POS	0x1
#define EDMP_HOST_INT_SELF_TEST_DONE_POS	0x2
#define EDMP_HOST_INT_SA_DONE_POS	0x4
#define EDMP_HOST_INT_BASIC_SMD_DET_POS	0x5
#define RAM_PATCHES_PRGM_RAM_BASE	0x500


#ifdef __cplusplus
}
#endif

#endif // __INV_IMU_EDMP_DEFS_H__
