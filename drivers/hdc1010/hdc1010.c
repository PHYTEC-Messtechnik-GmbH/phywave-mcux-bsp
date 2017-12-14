/*
 * Copyright (c) 2014,2017 PHYTEC Messtechnik GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the PHYTEC Messtechnik GmbH nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdbool.h>
#include "i2c_wrapper.h"
#include "hdc1010.h"

/**
 * Register Map
 */
#define HDC1010_TEMPERATURE        0x00
#define HDC1010_HUMIDITY           0x01
#define HDC1010_CONFG              0x02
#define HDC1010_SID1               0xFB
#define HDC1010_SID2               0xFC
#define HDC1010_SID3               0xFD
#define HDC1010_MANUFACTURER_ID    0xFE
#define HDC1010_DEVICE_ID          0xFF

/**
 * Configuration Register Description
 */
#define HDC1010_CONFG_RST          (1 << 15)
#define HDC1010_CONFG_HEAT         (1 << 13)
#define HDC1010_CONFG_SEQ_MOD      (1 << 12)
#define HDC1010_CONFG_BTST_LOW     (1 << 11)
#define HDC1010_CONFG_TRES_MSK     (1 << 10)
#define HDC1010_CONFG_TRES_11      (1 << 10)
#define HDC1010_CONFG_TRES_14      0
#define HDC1010_CONFG_HRES_MSK     (1 << 9 | 1 << 8)
#define HDC1010_CONFG_HRES_14      0
#define HDC1010_CONFG_HRES_11      (1 << 8)
#define HDC1010_CONFG_HRES_8       (1 << 9)

/**
 * Manufacturer and Device ID
 */
#define HDC1010_MID_VALUE          0x5449
#define HDC1010_DID_VALUE          0x1000

#define I2C_SPEED                  I2C_SPEED_FAST

int hdc1010_test(hdc1010_t *dev)
{
	uint8_t reg[3];
	uint16_t tmp;

	if (i2c_read_regs(dev->handle, dev->addr, HDC1010_MANUFACTURER_ID,
			  reg, 3) != 3)
		return -1;

	tmp = ((uint16_t)reg[0] << 8) | reg[1];
	if (tmp != HDC1010_MID_VALUE)
		return -1;

	return 0;
}

int hdc1010_init(hdc1010_t *dev, i2c_rtos_handle_t *handle, uint8_t address)
{
	uint8_t reg[2];

	/* write device descriptor */
	dev->handle = handle;
	dev->addr = address;
	dev->initialized = false;

	if (hdc1010_test(dev))
		return -2;

	/* set 14 bit resolution for both sensors and sequence mode */
	uint16_t tmp = HDC1010_CONFG_SEQ_MOD;
	reg[0] = (uint8_t)(tmp >> 8);
	reg[1] = (uint8_t)tmp;

	if (i2c_write_regs(dev->handle, dev->addr, HDC1010_CONFG, reg, 2) != 2)
		return -3;
	dev->initialized = true;

	return 0;
}

int hdc1010_reset(hdc1010_t *dev)
{
	uint8_t reg[2];
	uint16_t tmp = HDC1010_CONFG_RST;

	reg[0] = (uint8_t)(tmp >> 8);
	reg[1] = (uint8_t)tmp;
	dev->initialized = false;

	if (i2c_write_regs(dev->handle, dev->addr, HDC1010_CONFG, reg, 2) != 2)
		return -1;

	return 0;
}

int hdc1010_startmeasure(hdc1010_t *dev)
{
	if (dev->initialized == false)
		return -1;

	/* Trigger the measurements by executing a write access
	 * to the address 0x00 (HDC1010_TEMPERATURE).
	 * Conversion Time is 6.50ms by 14 bit resolution.
	 */
	if (i2c_write_byte(dev->handle, dev->addr, HDC1010_TEMPERATURE) != 1)
		return -1;

	return 0;
}

int hdc1010_read(hdc1010_t *dev, uint16_t *rawtemp, uint16_t *rawhum)
{
	uint8_t buf[4];

	if (dev->initialized == false)
		return -1;

	if (i2c_read_bytes(dev->handle, dev->addr, buf, 4) != 4)
		return -1;
	/* Register bytes are sent MSB first. */
	*rawtemp = ((uint16_t)buf[0] << 8) | buf[1];
	*rawhum = ((uint16_t)buf[2] << 8) | buf[3];

	return 0;
}


void hdc1010_convert(uint16_t rawtemp, uint16_t rawhum,  int *temp, int *hum)
{
	/* calculate temperature*100 [°C] */
	*temp = (int)((((int32_t)rawtemp * 16500) >> 16) - 4000);
	/*DEBUG("hdc1010 : T: %d\n", *temp);*/

	/* calculate relative humidity*100 [%RH] */
	*hum = (int)(((int32_t)rawhum * 10000) >> 16);
	/*DEBUG("hdc1010 : RH: %d\n", *hum);*/
}
