/**
 ******************************************************************************
 * @addtogroup PIOS PIOS Core hardware abstraction layer
 * @{
 * @addtogroup PIOS_LPS331AP LPS331AP Functions
 * @brief Deals with the hardware interface to the magnetometers and accelerometer
 * @{
 * @file       pios_lps331ap.c
 * @author     vasily dybala 2013.
 * @brief      LPS331AP Magnetic Sensor Functions from AHRS
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

#ifdef PIOS_INCLUDE_LPS331AP

#define LPS331AP_ADDR (0xBB >> 1)

const struct pios_lps331ap_cfg *cfg = 0;
static uint32_t i2c_id = 0;
static struct pios_lps331ap_data data;
static bool data_ready = false;

/**
 * @brief Reads one or more bytes into a buffer
 * \param[in] address LPS331AP register address (depends on size)
 * \param[out] buffer destination buffer
 * \param[in] len number of bytes which should be read
 * \return 0 if operation was successful
 * \return -1 if error during I2C transfer
 * \return -2 if unable to claim i2c device
 */
static int32_t PIOS_LPS331AP_Read(uint8_t address, uint8_t *buffer, uint8_t len)
{
    uint8_t addr_buffer[] = {
        address,
    };

    const struct pios_i2c_txn txn_list[] = {
        {
            .info = __func__,
            .addr = LPS331AP_ADDR,
            .rw   = PIOS_I2C_TXN_WRITE,
            .len  = sizeof(addr_buffer),
            .buf  = addr_buffer,
        }
        ,
        {
            .info = __func__,
            .addr = LPS331AP_ADDR,
            .rw   = PIOS_I2C_TXN_READ,
            .len  = len,
            .buf  = buffer,
        }
    };

    return PIOS_I2C_Transfer(i2c_id, txn_list, NELEMENTS(txn_list));
}

/**
 * @brief Writes one or more bytes to the LPS331AP
 * \param[in] address Register address
 * \param[in] buffer source buffer
 * \return 0 if operation was successful
 * \return -1 if error during I2C transfer
 * \return -2 if unable to claim i2c device
 */
static int32_t PIOS_LPS331AP_Write(uint8_t address, uint8_t buffer)
{
    uint8_t dataBuffer[] = {
        address,
        buffer,
    };

    const struct pios_i2c_txn txn_list[] = {
        {
            .info = __func__,
            .addr = LPS331AP_ADDR,
            .rw   = PIOS_I2C_TXN_WRITE,
            .len  = sizeof(dataBuffer),
            .buf  = dataBuffer,
        }
        ,
    };

    ;
    return PIOS_I2C_Transfer(i2c_id, txn_list, NELEMENTS(txn_list));
}

/**
 * @brief Initialize the lps331ap barometer sensor.
 * @return none
 */
void PIOS_LPS331AP_Init(uint32_t i2c_id_, const struct pios_lps331ap_cfg *cfg_)
{
	i2c_id = i2c_id_;
    cfg = cfg_;

    // deactivate barometer (before setting odr)
    while (PIOS_LPS331AP_Write(LPS331AP_CTRL_REG1_ADDR, 0x00) != 0)
    {
    	continue;
    }

    // Set baro and temp average samples
	while (PIOS_LPS331AP_Write(LPS331AP_RES_CONF_ADDR, cfg->baro_avg | cfg->temp_avg) != 0)
	{
		continue;
	}

    // set odr and activate barometer.
    while (PIOS_LPS331AP_Write(LPS331AP_CTRL_REG1_ADDR,
    		LPS331AP_CTRL_REG1_ACTIVE_MODE | LPS331AP_CTRL_REG1_BLOCK_UPDATE | cfg->odr) != 0)
    {
    	continue;
    }
}

void PIOS_LPS331AP_ObtainData()
{
	data_ready = false;

	// wait for data available
	uint8_t status = 0;
	while (PIOS_LPS331AP_Read(LPS331AP_STATUS_REG_ADDR, &status, 1) != 0 || (status & LPS331AP_STATUS_REG_BARO_AVAILABLE) != LPS331AP_STATUS_REG_BARO_AVAILABLE)
	{
		continue;
	}

	// obtain temp data
	if ((status & LPS331AP_STATUS_REG_TEMP_AVAILABLE) == LPS331AP_STATUS_REG_TEMP_AVAILABLE)
	{
		uint8_t tempbuff[2];
		while (PIOS_LPS331AP_Read(LPS331AP_TEMP_OUT_LSB_ADDR, (uint8_t*)tempbuff, 1) != 0)
		{
			continue;
		}

		while (PIOS_LPS331AP_Read(LPS331AP_TEMP_OUT_LSB_ADDR, (uint8_t*)(tempbuff + 1), 1) != 0)
		{
			continue;
		}

		uint16_t rawTemp = 0;
		rawTemp = (tempbuff[1] << 8) + tempbuff[0];
		data.temp = 42.5f + rawTemp / 480.0f;
	}

	// obtain baro data
	uint8_t barobuff[3];
	while (PIOS_LPS331AP_Read(LPS331AP_BARO_OUT_LSB_ADDR, (uint8_t*)barobuff, 1) != 0)
	{
		continue;
	}

	while (PIOS_LPS331AP_Read(LPS331AP_BARO_OUT_LSB_ADDR + 1, (uint8_t*)(barobuff + 1), 1) != 0)
	{
		continue;
	}

	while (PIOS_LPS331AP_Read(LPS331AP_BARO_OUT_LSB_ADDR + 2, (uint8_t*)(barobuff + 2), 1) != 0)
	{
		continue;
	}

	uint32_t rawBaro = 0;
	rawBaro = (barobuff[2] << 16) + (barobuff[1] << 8) + barobuff[0];
	data.baro = rawBaro / 4096.0f / 10.0f; // mbar to kPa

	data_ready = true;
}

int32_t PIOS_LPS331AP_ReadBaro(struct pios_lps331ap_data *data_)
{
	if (!data_ready)
		return -1;

	*data_ = data;
	return 0;
}

static const uint32_t defaultTimeout = 40;

uint32_t PIOS_LPS331AP_GetUpdateTimeoutuS()
{
	if (cfg == 0)
		return defaultTimeout * 1000;

	switch (cfg->odr)
	{
	case LPS331AP_ODR_BARO_7HZ_TEMP_1HZ:
		return 1000000 / 7;

	case LPS331AP_ODR_BARO_7HZ_TEMP_7HZ:
		return 1000000 / 7;

	case LPS331AP_ODR_BARO_12HZ_TEMP_12HZ:
		return 1000000 / 12;

	case LPS331AP_ODR_BARO_12HZ_TEMP_1HZ:
		return 1000000 / 12;

	case LPS331AP_ODR_BARO_25HZ_TEMP_1HZ:
		return 1000000 / 25;

	case LPS331AP_ODR_BARO_25HZ_TEMP_25HZ:
		return 1000000 / 25;

	case LPS331AP_ODR_BARO_1HZ_TEMP_1HZ:
		return 1000000 / 1;

	case LPS331AP_ODR_ONE_SHOOT:
		return defaultTimeout * 1000;
	}

	return defaultTimeout * 1000;
}

int32_t PIOS_LPS331AP_Test()
{
	uint8_t val;
	while(PIOS_LPS331AP_Read(LPS331AP_WHO_AM_I_ADDR, &val, 1) != 0)
	{
		continue;
	}

	PIOS_DEBUG_Assert(val == 0xBB);
	return 0;
}

#endif /* PIOS_INCLUDE_LPS331AP */

/**
 * @}
 * @}
 */
