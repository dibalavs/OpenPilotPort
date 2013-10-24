/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_L3G4200D L3G4200D Functions
 * @brief Deals with the hardware interface to the L3G4200D 3-axis gyroscope
 * @{
 *
 * @file       pios_L3G4200D.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      PiOS BMC050 digital accelerometer driver.
 *                 - Driver for the L3G4200D digital gyroscope on the SPI bus.
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#ifndef PIOS_L3G4200D_H
#define PIOS_L3G4200D_H

#include <pios.h>

// Gyroscope addresses
#define L3G_CTRL_REG1_ADDR 0x20
#define L3G_CTRL_REG2_ADDR 0x21
#define L3G_CTRL_REG3_ADDR 0x22
#define L3G_CTRL_REG4_ADDR 0x23
#define L3G_CTRL_REG5_ADDR 0x24
#define L3G_OUT_TEMP_ADDR  0x26
#define L3G_STATUS_REG_ADDR  0x27
#define L3G_OUT_X_LBS_ADDR 0x28
#define L3G_OUT_Y_LBS_ADDR 0x2A
#define L3G_OUT_Z_LBS_ADDR 0x2C

enum { L3G_CTRL_REG1_X_EN = 0x01,
	   L3G_CTRL_REG1_Y_EN = 0x02,
	   L3G_CTRL_REG1_Z_EN = 0x04,
	   L3G_CTRL_REG1_POWER = 0x08 };

enum l3g_ctrl_reg1_odr_cutoff { L3G_CTRL_REG1_ODR_100HZ_CUTOF_12_5HZ = (0x00 << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_100HZ_CUTOF_25HZ   = (0x01 << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_100HZ_CUTOF_25HZ1  = (0x02 << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_100HZ_CUTOF_25HZ2  = (0x03 << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_200HZ_CUTOF_12_5HZ = (0x04 << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_200HZ_CUTOF_25HZ   = (0x05 << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_200HZ_CUTOF_50HZ   = (0x06 << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_200HZ_CUTOF_70HZ   = (0x07 << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_400HZ_CUTOF_20HZ   = (0x08 << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_400HZ_CUTOF_25HZ   = (0x09 << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_400HZ_CUTOF_50HZ   = (0x0A << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_400HZ_CUTOF_110HZ  = (0x0B << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_800HZ_CUTOF_30HZ   = (0x0C << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_800HZ_CUTOF_35HZ   = (0x0D << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_800HZ_CUTOF_50HZ   = (0x0E << 4),
	   	   	   	   	   	   	    L3G_CTRL_REG1_ODR_800HZ_CUTOF_110HZ  = (0x0F << 4) };

enum l3g_ctrl_reg2_cutoff_freq { L3G_CTRL_REG2_CUTOF_8HZ  = 0x00,
								 L3G_CTRL_REG2_CUTOF_8HZ  = 0x01,
	   	   	   	   	   	   	     L3G_CTRL_REG2_CUTOF_4HZ  = 0x02,
	   	   	   	   	   	   	     L3G_CTRL_REG2_CUTOF_2HZ  = 0x03,
	   	   	   	   	   	   	     L3G_CTRL_REG2_CUTOF_1HZ  = 0x04,
	   	   	   	   	   	   	     L3G_CTRL_REG2_CUTOF_0_5HZ  = 0x05,
	   	   	   	   	   	   	     L3G_CTRL_REG2_CUTOF_0_2HZ  = 0x06,
	   	   	   	   	   	   	     L3G_CTRL_REG2_CUTOF_0_1HZ  = 0x07,
	   	   	   	   	   	   	     L3G_CTRL_REG2_CUTOF_0_05HZ  = 0x08,
	   	   	   	   	   	   	     L3G_CTRL_REG2_CUTOF_0_02HZ  = 0x09 };



enum l3g_ctrl_reg4_scale { L3G_CTRL_REG4_200DPS = 0x00 << 5,
						   L3G_CTRL_REG4_500DPS = 0x01 << 5,
						   L3G_CTRL_REG4_2000DPS = 0x10 << 5,
						   L3G_CTRL_REG4_2000DPS1 = 0x11 << 5};

enum { L3G_CTRL_REG5_HP_FILTER_EN = 0x10 };

enum { L3G_STATUS_REG_XYZDA = 0x08 };

struct pios_l3g_gyro_data {
    float32_t gyro_x; // g
    float32_t gyro_y; // g
    float32_t gyro_z; // g
    float32_t temperature; // C
};

struct pios_l3g_cfg {
    enum l3g_ctrl_reg1_odr_cutoff bandwidth;
    enum l3g_ctrl_reg2_cutoff_freq cutoff;
    enum l3g_ctrl_reg4_scale scale;
};

/* Public Functions */
extern int32_t PIOS_L3G4200D_Init(uint32_t spi_id, uint32_t slave_num, const struct pios_l3g_cfg *cfg);

// Gyro functions
extern float PIOS_L3G4200D_GetScale();
extern int32_t PIOS_L3G4200D_ReadGyro(struct pios_bmc050_accel_data *data);
extern void PIOS_L3G4200D_ObtainData();
extern uint32_t PIOS_L3G4200D_GetUpdateGyroTimeoutuS();
extern int32_t PIOS_L3G4200D_GyroTest();

#endif /* PIOS_L3G4200D_H */

/**
 * @}
 * @}
 */
