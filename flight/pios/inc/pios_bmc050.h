/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_BMC050 BMC050 Functions
 * @brief Deals with the hardware interface to the BMC050 6-axis accelerometer and magnetometer
 * @{
 *
 * @file       pios_bmc050.h
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      PiOS BMC050 digital accelerometer driver.
 *                 - Driver for the BMC050 digital accelerometer on the SPI bus.
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

#ifndef PIOS_BMC050_H
#define PIOS_BMC050_H

#include <pios.h>

// Accelerometer addresses
#define BMC_ACCEL_CHIPID_ADDR    0x00
#define BMC_ACCEL_TEMP_ADDR      0x08
#define BMC_ACCEL_X_LSB_ADDR     0x02
#define BMC_ACCEL_Y_LSB_ADDR     0x04
#define BMC_ACCEL_Z_LSB_ADDR     0x06
#define BMC_ACCEL_ORIENT_ADDR    0x0C
#define BMC_ACCEL_G_RANGE_ADDR   0x0F
#define BMC_ACCEL_BANDWIDTH_ADDR 0x10
#define BMC_ACCEL_POWERMODE_ADDR 0x11
#define BMC_ACCEL_CONFIG_ADDR    0x13
#define BMC_ACCEL_RESET_ADDR     0x14

// Magnetometer addresses
#define BMC_MAG_CHIPID_ADDR      0x40
#define BMC_MAG_X_LSB_ADDR       0x42
#define BMC_MAG_Y_LSB_ADDR       0x44
#define BMC_MAG_Z_LSB_ADDR       0x46
#define BMC_MAG_HALL_RES_LSB_ADDR 0x48 /*and Data ready bit*/
#define BMC_MAG_POWER             0x4B /*power, soft reset*/
#define BMC_MAG_OP_MODE_BANDWIDTH 0x4C
#define BMC_MAG_REPETITION_XY     0x51
#define BMC_MAG_REPETITION_Z      0x52

enum { BMC_ACCEL_RESET_VAL = 0xB6 };

enum { BMC_MAG_RESET_VAL = 0x83 };

enum { BMC_ACCEL_DATA_READY_BIT = 0x01 };

enum { BMC_MAG_DATA_READY_BIT = 0x01 };

/* Accel range  */
enum bmc050_accel_range { BMC_ACCEL_RANGE_2G   = 0x03,
                          BMC_ACCEL_RANGE_4G   = 0x05,
					      BMC_ACCEL_RANGE_8G   = 0x08,
                          BMC_ACCEL_RANGE_16G  = 0x0C };

/* Measurement bandwidth */
enum bmc050_accel_bandwidth { BMC_ACCEL_BW_8HZ       = 0x08,
                              BMC_ACCEL_BW_16HZ      = 0x09,
                              BMC_ACCEL_BW_31HZ      = 0x0A,
                              BMC_ACCEL_BW_63HZ      = 0x0B,
                              BMC_ACCEL_BW_125HZ     = 0x0C,
                              BMC_ACCEL_BW_250HZ     = 0x0D,
                              BMC_ACCEL_BW_500HZ     = 0x0E,
                              BMC_ACCEL_BW_1000HZ    = 0x0F };

enum bmc050_mag_odr { BMC_MAG_ODR_2HZ = (0x01 << 3),
					  BMC_MAG_ODR_6HZ = (0x02 << 3),
					  BMC_MAG_ODR_8HZ = (0x03 << 3),
					  BMC_MAG_ODR_10HZ = 0x00,
					  BMC_MAG_ODR_15HZ = (0x04 << 3),
					  BMC_MAG_ODR_20HZ = (0x05 << 3),
					  BMC_MAG_ODR_25HZ = (0x06 << 3),
					  BMC_MAG_ODR_25HZ = (0x07 << 3)
};

enum bmc050_mag_operation_mode { BMC_MAG_OP_NORMAL   = 0x00,
								 BMC_MAG_OP_FORCED   = 0x02,
								 BMC_MAG_OP_SLEEP    = 0x06 };
struct pios_bmc050_accel_data {
    float32_t accel_x; // g
    float32_t accel_y; // g
    float32_t accel_z; // g
    float32_t accel_temperature; // C
};

struct pios_bmc050_mag_data {
	float32_t accel_temperature; // C
    float32_t mag_hall_resistance;
    float32_t mag_x; // mT
	float32_t mag_y; // mT
	float32_t mag_z; // mT
};

struct pios_bmc050_cfg {
    enum bmc050_accel_bandwidth accel_bandwidth;
    enum bmc050_accel_range accel_range;

    enum bmc050_mag_odr mag_odr;
    int8_t mag_rep_xy;
    int8_t mag_rep_z;
};

/* Public Functions */
extern int32_t PIOS_BMC050_Init(uint32_t spi_id, uint32_t slave_num_accel, uint32_t slave_num_mag, const struct pios_bmc050_cfg *cfg);

// Accelerometer functions
extern float PIOS_BMC050_GetAccelScale();
extern int32_t PIOS_BMC050_ReadAccel(struct pios_bmc050_accel_data *data);
extern void PIOS_BMC050_ObtainAccelData();
extern uint32_t PIOS_BMC050_GetUpdateAccelTimeoutuS();
extern int32_t PIOS_BMC050_AccelTest();

// Magnetometer functions
extern float PIOS_BMC050_GetMagScale();
extern void PIOS_BMC050_ObtainMagData();
extern uint32_t PIOS_BMC050_GetUpdateMagTimeoutuS();
extern int32_t PIOS_BMC050_ReadMag(struct pios_bmc050_mag_data *data);
extern int32_t PIOS_BMC050_MagTest();

#endif /* PIOS_BMC050_H */

/**
 * @}
 * @}
 */
