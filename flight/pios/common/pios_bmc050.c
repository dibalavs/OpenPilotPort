/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_BMC050 BMC050 Functions
 * @brief Deals with the hardware interface to the magnetometers and accelerometer
 * @{
 * @file       pios_bmc050.c
 * @author     vasily dybala 2013.
 * @brief      BMC050 Magnetic Sensor Functions from AHRS
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

#ifdef PIOS_INCLUDE_BMC050

/* Global Variables */

/* Local Types */

/* Local Variables */
//volatile bool pios_bmc050_data_ready;

static int32_t PIOS_BMC050_ConfigAccel(const struct pios_bmc050_cfg *cfg);
static int32_t PIOS_BMC050_ConfigMag(const struct pios_bmc050_cfg *cfg);

//static const struct pios_bmc050_cfg *dev_cfg;

struct bmc050_dev {
    uint32_t spi_id;
    uint32_t slave_num;
    const struct pios_bmc050_cfg *cfg;
} devAccel, devMag;

struct pios_bmc050_raw_data {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int8_t accel_temperature;

    int8_t mag_hall_resistance;
    int16_t mag_x;
	int16_t mag_y;
	int16_t mag_z;
};

static struct bmc050_dev* dev = 0;
static struct pios_bmc050_accel_data accel_data;
static struct pios_bmc050_mag_data mag_data;
static bool data_ready = false;

/**
 * @brief Validate the handle to the spi device
 * @returns 0 for valid device or -1 otherwise
 */
static int32_t PIOS_BMC050_Validate(struct bmc050_dev *vdev)
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
int32_t PIOS_BMC050_ClaimBus()
{
    if (PIOS_BMC050_Validate(dev) != 0) {
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
int32_t PIOS_BMC050_ReleaseBus()
{
    if (PIOS_BMC050_Validate(dev) != 0) {
        return -1;
    }
    PIOS_SPI_RC_PinSet(dev->spi_id, dev->slave_num, 1);

    return PIOS_SPI_ReleaseBus(dev->spi_id);
} 

/**
 * @brief Read a register from BMC050
 * @returns The register value or -1 if failure to get bus
 * @param reg[in] Register address to be read
 */
int32_t PIOS_BMC050_GetReg(uint8_t reg)
{
    if (PIOS_BMC050_Validate(dev) != 0) {
        return -1;
    }

    uint8_t data;

    if (PIOS_BMC050_ClaimBus() != 0) {
        return -1;
    }

    PIOS_SPI_TransferByte(dev->spi_id, (0x80 | reg)); // request byte
    data = PIOS_SPI_TransferByte(dev->spi_id, 0); // receive response

    PIOS_BMC050_ReleaseBus();
    return data;
}

/**
 * @brief Write a BMC050 register.  EEPROM must be unlocked before calling this function.
 * @return none
 * @param reg[in] address of register to be written
 * @param data[in] data that is to be written to register
 */
int32_t PIOS_BMC050_SetReg(uint8_t reg, uint8_t data)
{
    if (PIOS_BMC050_ClaimBus() != 0) {
        return -1;
    }

    PIOS_SPI_TransferByte(dev->spi_id, 0x7f & reg);
    PIOS_SPI_TransferByte(dev->spi_id, data);

    PIOS_BMC050_ReleaseBus();

    return 0;
}

void PIOS_BMC050_SetAccel()
{
	dev = &devAccel;
}

void PIOS_BMC050_SetMag()
{
	dev = &devMag;
}

/**
 * @brief Initialize the bmc050 magnetometer sensor.
 * @return none
 */
void PIOS_BMC050_Init(uint32_t spi_id, uint32_t slave_num_accel, uint32_t slave_num_mag, const struct pios_bmc050_cfg *cfg)
{
	devAccel.cfg = cfg;
	devAccel.spi_id = spi_id;
	devAccel.slave_num = slave_num_accel;

	devMag.cfg = cfg;
	devMag.slave_num = slave_num_mag;
	devMag.spi_id = spi_id;

	PIOS_BMC050_SetAccel();
	PIOS_BMC050_ReleaseBus();

	PIOS_BMC050_SetMag();
	PIOS_BMC050_ReleaseBus();

    int32_t val = PIOS_BMC050_ConfigAccel(cfg);
    PIOS_Assert(val == 0);

    val = PIOS_BMC050_ConfigMag(cfg);
    PIOS_Assert(val == 0);

    //pios_bmc050_data_ready = false;
}

/**
 * @brief Initialize the BMC050 magnetometer and accelerometer sensor
 * \return none
 * \param[in] PIOS_BMC050_ConfigTypeDef struct to be used to configure sensor.
*/
static int32_t PIOS_BMC050_ConfigAccel(const struct pios_bmc050_cfg *cfg)
{
	PIOS_BMC050_SetAccel();

	//reboot
	PIOS_BMC050_SetReg(BMC_ACCEL_RESET_ADDR, BMC_ACCEL_RESET_VAL);
	PIOS_DELAY_WaitmS(100);

	// normal mode
	PIOS_BMC050_SetReg(BMC_ACCEL_POWERMODE_ADDR, 0);

	// enable filtering
	PIOS_BMC050_SetReg(BMC_ACCEL_CONFIG_ADDR, 0);

	// bandwidth
	PIOS_BMC050_SetReg(BMC_ACCEL_BANDWIDTH_ADDR, cfg->accel_bandwidth);

	// g range
	PIOS_BMC050_SetReg(BMC_ACCEL_G_RANGE_ADDR, cfg->accel_range);

    return 0;
}

static int32_t PIOS_BMC050_ConfigMag(const struct pios_bmc050_cfg *cfg)
{
	PIOS_BMC050_SetMag();

	// reset
	PIOS_BMC050_SetReg(BMC_MAG_POWER, BMC_MAG_RESET_VAL);
	PIOS_DELAY_WaitmS(100);
	PIOS_BMC050_SetReg(BMC_MAG_POWER, 1);

	// ODR 20hz, normal operation mode
	PIOS_BMC050_SetReg(BMC_MAG_OP_MODE_BANDWIDTH, (cfg->mag_odr) | BMC_MAG_OP_NORMAL);

	// set repetition;
	PIOS_BMC050_SetReg(BMC_MAG_REPETITION_XY, (cfg->mag_rep_xy - 1) / 2);
	PIOS_BMC050_SetReg(BMC_MAG_REPETITION_Z, cfg->mag_rep_z - 1);

	return 0;
}

static float PIOS_BMC050_RangeToValue()
{
	switch (dev->cfg->accel_range)
	{
	case BMC_ACCEL_RANGE_2G:
		return 2.0f;

	case BMC_ACCEL_RANGE_4G:
		return 4.0f;

	case BMC_ACCEL_RANGE_8G:
		return 8.0f;

	case BMC_ACCEL_RANGE_16G:
		return 16.0f;

	default:
		PIOS_Assert(false);
		return 0;
	}
}

const float mg_per_lsb = 1.953125f;
const float temp_per_lsb = 0.5f;
void NormalizeAccelData(struct pios_bmc050_raw_data *raw, struct pios_bmc050_accel_data *data)
{
	const float coeff_g = PIOS_BMC050_RangeToValue() * mg_per_lsb * 0.001f;
	data->accel_x = coeff_g * raw->accel_x;
	data->accel_y = coeff_g * raw->accel_y;
	data->accel_z = coeff_g * raw->accel_z;
	data->accel_temperature = temp_per_lsb * (raw->accel_temperature + 24.0f * 2.0f);

	// TODO do temperature compensation.
}

const float mt_per_lsb_xy = 2000.0f / ((1 << 14) - 1);
const float mt_per_lsb_z = 5000.0f / ((1 << 16) - 1);
void NormalizeMagData(struct pios_bmc050_raw_data *raw, struct pios_bmc050_mag_data *data)
{
	data->mag_x = raw->mag_x * mt_per_lsb_xy;
	data->mag_y = raw->mag_y * mt_per_lsb_xy;
	data->mag_z = raw->mag_z * mt_per_lsb_z;

	data->accel_temperature = temp_per_lsb * (raw->accel_temperature + 24.0f * 2.0f);
	// TODO add temperature compensation.
}

void PIOS_BMC050_ObtainAccelData()
{
	data_ready = false;
	int32_t rx = 0;

	PIOS_BMC050_SetAccel();

	struct pios_bmc050_raw_data raw;

	// X coordinate raw
	while(((rx = PIOS_BMC050_GetReg(BMC_ACCEL_X_LSB_ADDR)) & BMC_ACCEL_DATA_READY_BIT) == 0);
	raw.accel_x = (PIOS_BMC050_GetReg(BMC_ACCEL_X_LSB_ADDR + 1) << 2) | (rx >> 6);

	// Y coordinate raw
	while(((rx = PIOS_BMC050_GetReg(BMC_ACCEL_Y_LSB_ADDR)) & BMC_ACCEL_DATA_READY_BIT) == 0);
	raw.accel_y = (PIOS_BMC050_GetReg(BMC_ACCEL_Y_LSB_ADDR + 1) << 2) | (rx >> 6);

	// Z coordinate raw
	while(((rx = PIOS_BMC050_GetReg(BMC_ACCEL_Z_LSB_ADDR)) & BMC_ACCEL_DATA_READY_BIT) == 0);
	raw.accel_z = (PIOS_BMC050_GetReg(BMC_ACCEL_Z_LSB_ADDR + 1) << 2) | (rx >> 6);

	// raw temperature
	raw.accel_temperature = PIOS_BMC050_GetReg(BMC_ACCEL_TEMP_ADDR);

	NormalizeAccelData(&raw, &accel_data);

	data_ready = true;
}

void PIOS_BMC050_ObtainMagData()
{
	data_ready = false;
	int32_t rx = 0;
	struct pios_bmc050_raw_data raw;

	PIOS_BMC050_SetAccel();

	// raw temperature
	raw.accel_temperature = PIOS_BMC050_GetReg(BMC_ACCEL_TEMP_ADDR);

	PIOS_BMC050_SetMag();

	// raw hall resistance;
	while(((rx = PIOS_BMC050_GetReg(BMC_MAG_HALL_RES_LSB_ADDR)) & BMC_MAG_DATA_READY_BIT )== 0);
	raw.mag_hall_resistance = (PIOS_BMC050_GetReg(BMC_MAG_HALL_RES_LSB_ADDR + 1) << 6) | (rx >> 2);

	// x coord raw
	rx = PIOS_BMC050_GetReg(BMC_MAG_X_LSB_ADDR);
	raw.mag_x = (PIOS_BMC050_GetReg(BMC_MAG_X_LSB_ADDR + 1) << 5) | (rx >> 3);

	// y coord raw
	rx = PIOS_BMC050_GetReg(BMC_MAG_Y_LSB_ADDR);
	raw.mag_y = (PIOS_BMC050_GetReg(BMC_MAG_Y_LSB_ADDR + 1) << 5) | (rx >> 3);

	// z coord raw
	rx = PIOS_BMC050_GetReg(BMC_MAG_Z_LSB_ADDR);
	raw.mag_z = (PIOS_BMC050_GetReg(BMC_MAG_Z_LSB_ADDR + 1) << 7) | (rx >> 1);

	NormalizeMagData(&raw, &mag_data);

	data_ready = true;
}

int32_t PIOS_BMC050_ReadAccel(struct pios_bmc050_accel_data *data)
{
	if (!data_ready)
		return -1;

	*data = accel_data;
	return 0;
}

int32_t PIOS_BMC050_ReadMag(struct pios_bmc050_mag_data *data)
{
	if (!data_ready)
		return -1;

	*data = mag_data;
	return 0;
}

float PIOS_BMC050_GetAccelScale()
{
	return 1;
}

float PIOS_BMC050_GetMagScale()
{
	return 1;
}

static const uint32_t defaultTimeout = 40;

uint32_t PIOS_BMC050_AccelBandwidthToPeriodMs(enum bmc050_accel_bandwidth bw)
{
	switch (bw)
	{
	case BMC_ACCEL_BW_16HZ:
		return 1000 / 16;

	case BMC_ACCEL_BW_31HZ:
		return 1000 / 31;

	case BMC_ACCEL_BW_63HZ:
		return 1000 / 63;

	case BMC_ACCEL_BW_125HZ:
		return 1000 / 125;

	case BMC_ACCEL_BW_250HZ:
		return 1000 / 250;

	case BMC_ACCEL_BW_500HZ:
		return 1000 / 500;

	case BMC_ACCEL_BW_1000HZ:
		return 1000 / 1000;

	case BMC_ACCEL_BW_8HZ:
		return 1000 / 8;
	}

	return defaultTimeout;
}

uint32_t PIOS_BMC050_GetUpdateMagTimeoutuS()
{
	if (dev == 0)
		return defaultTimeout * 1000;

	switch (dev->cfg->mag_odr)
	{
	case BMC_MAG_ODR_20HZ:
		return 1000000 / 20;

	case BMC_MAG_ODR_10HZ:
		return 1000000 / 10;

	case BMC_MAG_ODR_2HZ:
		return 1000000 / 2;

	case BMC_MAG_ODR_6HZ:
		return 1000000 / 6;

	case BMC_MAG_ODR_8HZ:
		return 1000000 / 8;

	case BMC_MAG_ODR_15HZ:
		return 1000000 / 15;

	case BMC_MAG_ODR_25HZ:
		return 1000000 / 25;

	case BMC_MAG_ODR_30HZ:
			return 1000000 / 30;
	}

	return defaultTimeout * 1000;
}

uint32_t PIOS_BMC050_GetUpdateAccelTimeoutuS()
{
	if (dev == 0)
		return defaultTimeout * 1000;

	switch (dev->cfg->accel_bandwidth)
	{
	case BMC_ACCEL_BW_16HZ:
		return 1000000 / 16;

	case BMC_ACCEL_BW_31HZ:
		return 1000000 / 31;

	case BMC_ACCEL_BW_63HZ:
		return 1000000 / 63;

	case BMC_ACCEL_BW_125HZ:
		return 1000000 / 125;

	case BMC_ACCEL_BW_250HZ:
		return 1000000 / 250;

	case BMC_ACCEL_BW_500HZ:
		return 1000000 / 500;

	case BMC_ACCEL_BW_1000HZ:
		return 1000000 / 1000;

	case BMC_ACCEL_BW_8HZ:
		return 1000000 / 8;
	}

	return defaultTimeout * 1000;
}

int32_t PIOS_BMC050_AccelTest()
{
	PIOS_BMC050_SetMag();
	PIOS_BMC050_ReleaseBus();

	PIOS_BMC050_SetAccel();
	uint32_t rx = PIOS_BMC050_GetReg(BMC_ACCEL_CHIPID_ADDR);
	PIOS_Assert(rx == 0x03);

	return 0;
}

int32_t PIOS_BMC050_MagTest()
{
	PIOS_BMC050_SetAccel();
	PIOS_BMC050_ReleaseBus();

	PIOS_BMC050_SetMag();
	PIOS_BMC050_SetReg(BMC_MAG_POWER, 1);
	uint32_t rx = PIOS_BMC050_GetReg(BMC_MAG_CHIPID_ADDR);
	PIOS_Assert(rx == 0x32);

	return 0;
}

#endif /* PIOS_INCLUDE_BMC050 */

/**
 * @}
 * @}
 */
