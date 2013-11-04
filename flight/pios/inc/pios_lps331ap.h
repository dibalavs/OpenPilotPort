/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_LPS331AP LPS331AP Functions
 * @brief Deals with the hardware interface to the LPS331AP barometr
 * @{
 *
 * @file       pios_lps331ap.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      PiOS LPS331AP digital accelerometer driver.
 *                 - Driver for the LPS331AP barometr on the I2C bus.
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

#ifndef PIOS_LPS331AP_H
#define PIOS_LPS331AP_H

#include <pios.h>

#define LPS331AP_WHO_AM_I_ADDR 0x0F
#define LPS331AP_RES_CONF_ADDR 0x10
#define LPS331AP_CTRL_REG1_ADDR 0x20
#define LPS331AP_CTRL_REG2_ADDR 0x21
#define LPS331AP_STATUS_REG_ADDR 0x27

// 24 bit (3 bytes)
#define LPS331AP_BARO_OUT_LSB_ADDR 0x28
// 16 bit (2 bytes)
#define LPS331AP_TEMP_OUT_LSB_ADDR 0x2B

enum { LPS331AP_CTRL_REG1_ACTIVE_MODE = 0x80,
	   LPS331AP_CTRL_REG1_BLOCK_UPDATE = 0x04};

enum { LPS331AP_CTRL_REG2_ONE_SHOOT = 0x01,
	   LPS331AP_CTRL_REG2_RESET = 0x04 };

enum { LPS331AP_STATUS_REG_TEMP_AVAILABLE = 0x01,
	   LPS331AP_STATUS_REG_BARO_AVAILABLE = 0x02 };

enum lps331ap_ctrl_reg1_odr
{
	LPS331AP_ODR_ONE_SHOOT = (0x00 << 4),
	LPS331AP_ODR_BARO_1HZ_TEMP_1HZ = (0x01 << 4),
	LPS331AP_ODR_BARO_7HZ_TEMP_1HZ = (0x02 << 4),
	LPS331AP_ODR_BARO_12HZ_TEMP_1HZ = (0x03 << 4),
	LPS331AP_ODR_BARO_25HZ_TEMP_1HZ = (0x04 << 4),
	LPS331AP_ODR_BARO_7HZ_TEMP_7HZ = (0x05 << 4),
	LPS331AP_ODR_BARO_12HZ_TEMP_12HZ = (0x06 << 4),
	LPS331AP_ODR_BARO_25HZ_TEMP_25HZ = (0x07 << 4)
};

enum lps331ap_baro_average_items
{
	LPS331AP_BARO_AVG_1 = 0,
	LPS331AP_BARO_AVG_2 = 1,
	LPS331AP_BARO_AVG_4 = 2,
	LPS331AP_BARO_AVG_8 = 3,
	LPS331AP_BARO_AVG_16 = 4,
	LPS331AP_BARO_AVG_32 = 5,
	LPS331AP_BARO_AVG_64 = 6,
	LPS331AP_BARO_AVG_128 = 7,
	LPS331AP_BARO_AVG_256 = 8,
	LPS331AP_BARO_AVG_386 = 9,
	LPS331AP_BARO_AVG_512 = 10	// preferable
};

enum lps331ap_temp_average_items
{
	LPS331AP_TEMP_AVG_1 = (0 << 4),
	LPS331AP_TEMP_AVG_2 = (1 << 4),
	LPS331AP_TEMP_AVG_4 = (2 << 4),
	LPS331AP_TEMP_AVG_8 = (3 << 4),
	LPS331AP_TEMP_AVG_16 = (4 << 4),
	LPS331AP_TEMP_AVG_32 = (5 << 4),
	LPS331AP_TEMP_AVG_64 = (6 << 4),
	LPS331AP_TEMP_AVG_128 = (7 << 4), // preferable
};

struct pios_lps331ap_cfg {
    enum lps331ap_baro_average_items baro_avg;
    enum lps331ap_temp_average_items temp_avg;
    enum lps331ap_ctrl_reg1_odr odr;
};

struct pios_lps331ap_data {
	float baro; // kPa
	float temp; // C deg
};

extern void PIOS_LPS331AP_Init(uint32_t i2c_id, const struct pios_lps331ap_cfg *cfg);
extern void PIOS_LPS331AP_ObtainData();
extern int32_t PIOS_LPS331AP_ReadBaro(struct pios_lps331ap_data *data);
extern uint32_t PIOS_LPS331AP_GetUpdateTimeoutuS();
extern int32_t PIOS_LPS331AP_Test();

#endif /* PIOS_LPS331AP_H */

/**
 * @}
 * @}
 */
