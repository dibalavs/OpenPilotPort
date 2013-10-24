/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_L3G4200D L3G4200D Functions
 * @brief Deals with the hardware interface to the gyroscope
 * @{
 * @file       pios_L3G4200D.c
 * @author     vasily dybala 2013.
 * @brief      L3G4200D digital gyroscope
 * @see        The GNU Public License (GPL) Version 3
 *
 ******************************************************************************
 */
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

#include "pios.h"

#ifdef PIOS_INCLUDE_L3G4200D

struct l3g_dev {
    uint32_t spi_id;
    uint32_t slave_num;
    const struct pios_l3g_cfg *cfg;
} dev;

static struct pios_l3g_gyro_data data;
static bool data_ready = false;

/**
 * @brief Validate the handle to the spi device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t PIOS_L3G4200D_Validate(struct l3g_dev *vdev)
{
    if (vdev == NULL) {
        return -1;
    }
    if (vdev->spi_id == 0) {
        return -3;
    }
    return 0;
} 

/**
 * @brief Claim the SPI bus for the accel communications and select this chip
 * @return 0 if successful, -1 if unable to claim bus
 */
int32_t PIOS_L3G4200D_ClaimBus()
{
    if (PIOS_L3G_Validate(dev) != 0) {
        return -1;
    }

    if (PIOS_SPI_ClaimBus(dev->spi_id) != 0) {
        return -1;
    }

    PIOS_SPI_RC_PinSet(dev->spi_id, dev->slave_num, 0);

    return 0;
}

/**
 * @brief Release the SPI bus for the accel communications and end the transaction
 * @return 0 if successful
 */
int32_t PIOS_L3G4200D_ReleaseBus()
{
    if (PIOS_L3G_Validate(dev) != 0) {
        return -1;
    }
    PIOS_SPI_RC_PinSet(dev->spi_id, dev->slave_num, 1);

    return PIOS_SPI_ReleaseBus(dev->spi_id);
} 

/**
 * @brief Read a register from L3G4200D
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
int32_t PIOS_L3G4200D_GetReg(uint8_t reg)
{
    if (PIOS_L3G_Validate(dev) != 0) {
        return -1;
    }

    uint8_t data;

    if (PIOS_L3G_ClaimBus() != 0) {
        return -1;
    }

    PIOS_SPI_TransferByte(dev->spi_id, (0x80 | reg)); // request byte
    data = PIOS_SPI_TransferByte(dev->spi_id, 0); // receive response

    PIOS_L3G_ReleaseBus();
    return data;
}

/**
 * @brief Write a L3G register.  EEPROM must be unlocked before calling this function.
 * @return none
 * @param reg[in] address of register to be written
 * @param data[in] data that is to be written to register
 */
int32_t PIOS_L3G4200D_SetReg(uint8_t reg, uint8_t data)
{
    if (PIOS_L3G_ClaimBus() != 0) {
        return -1;
    }

    PIOS_SPI_TransferByte(dev->spi_id, 0x7f & reg);
    PIOS_SPI_TransferByte(dev->spi_id, data);

    PIOS_BMC050_ReleaseBus();

    return 0;
}

/**
 * @brief Initialize the bmc050 magnetometer sensor.
 * @return none
 */
void PIOS_L3G4200D_Init(uint32_t spi_id, uint32_t slave_num, const struct pios_l3g_cfg *cfg)
{
	dev.cfg = cfg;
	dev.spi_id = spi_id;
	dev.slave_num = slave_num;

	PIOS_L3G_SetReg(L3G_CTRL_REG1_ADDR, cfg->bandwidth
		| L3G_CTRL_REG1_X_EN | L3G_CTRL_REG1_Y_EN | L3G_CTRL_REG1_Z_EN | L3G_CTRL_REG1_POWER);
	PIOS_L3G_SetReg(L3G_CTRL_REG2_ADDR, cfg->cutoff);
	PIOS_L3G_SetReg(L3G_CTRL_REG3_ADDR, 0x00);
	PIOS_L3G_SetReg(L3G_CTRL_REG4_ADDR, cfg->scale);
	PIOS_L3G_SetReg(L3G_CTRL_REG5_ADDR, 0x00);
}

float PIOS_L3G4200D_GetScale()
{
	switch (dev->cfg->scale)
	{
	case L3G_CTRL_REG4_200DPS:
		return 0.00875f;

	case L3G_CTRL_REG4_500DPS:
		return 0.01750f;

	case L3G_CTRL_REG4_2000DPS:
	case L3G_CTRL_REG4_2000DPS1:
		return 0.070f;
	}
	return 0;
}

int32_t PIOS_L3G4200D_ReadGyro(struct pios_l3g_gyro_data *gyro_data)
{
	if (!data_ready)
		return -1;

	*gyro_data = data;
	return 0;
}

void PIOS_L3G4200D_ObtainData()
{
	data_ready = false;

	int32_t rx = 0;
	// wait for new data available.
	while((PIOS_L3G_GetReg(L3G_STATUS_REG_ADDR) & L3G_STATUS_REG_XYZDA) == 0);

	// temp coord
	uint32_t tempRaw = PIOS_L3G_GetReg(L3G_OUT_TEMP_ADDR);
	data.temperature = 50.0f - tempRaw;// todo add conversion

	data.gyro_x = PIOS_L3G_GetReg(L3G_OUT_X_LBS_ADDR) + (PIOS_L3G_GetReg(L3G_OUT_X_LBS_ADDR + 1) << 8 );
	data.gyro_y = PIOS_L3G_GetReg(L3G_OUT_Y_LBS_ADDR) + (PIOS_L3G_GetReg(L3G_OUT_Y_LBS_ADDR + 1) << 8 );
	data.gyro_z = PIOS_L3G_GetReg(L3G_OUT_Z_LBS_ADDR) + (PIOS_L3G_GetReg(L3G_OUT_Z_LBS_ADDR + 1) << 8 );

	data_ready = true;
}

const uint32_t defaultTimeout = 40; //ms

uint32_t PIOS_L3G4200D_GetUpdateGyroTimeoutuS()
{
	if (dev == 0)
		return defaultTimeout * 1000;

	switch (dev->cfg->bandwidth)
	{
	case L3G_CTRL_REG1_ODR_100HZ_CUTOF_12_5HZ:
	case L3G_CTRL_REG1_ODR_100HZ_CUTOF_25HZ:
	case L3G_CTRL_REG1_ODR_100HZ_CUTOF_25HZ1:
	case L3G_CTRL_REG1_ODR_100HZ_CUTOF_25HZ2:
		return 1000000 / 100;

	case L3G_CTRL_REG1_ODR_200HZ_CUTOF_12_5HZ:
	case L3G_CTRL_REG1_ODR_200HZ_CUTOF_25HZ:
	case L3G_CTRL_REG1_ODR_200HZ_CUTOF_50HZ:
	case L3G_CTRL_REG1_ODR_200HZ_CUTOF_70HZ:
		return 1000000 / 200;

	case L3G_CTRL_REG1_ODR_400HZ_CUTOF_20HZ:
	case L3G_CTRL_REG1_ODR_400HZ_CUTOF_25HZ:
	case L3G_CTRL_REG1_ODR_400HZ_CUTOF_50HZ:
	case L3G_CTRL_REG1_ODR_400HZ_CUTOF_110HZ:
		return 1000000 / 400;

	case L3G_CTRL_REG1_ODR_800HZ_CUTOF_30HZ:
	case L3G_CTRL_REG1_ODR_800HZ_CUTOF_35HZ:
	case L3G_CTRL_REG1_ODR_800HZ_CUTOF_50HZ:
	case L3G_CTRL_REG1_ODR_800HZ_CUTOF_110HZ:
		return 1000000 / 800;
	}

	return defaultTimeout * 1000;
}

int32_t PIOS_L3G4200D_GyroTest()
{
	return 0;
}

#endif /* PIOS_INCLUDE_BMC050 */

/**
 * @}
 * @}
 */
