/*
 * Copyright (c) 2015,2017 PHYTEC Messtechnik GmbH
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
#include "tcs37727.h"
#include "tcs37727_internal.h"

#define ENABLE_DEBUG   (0)

#define I2C_SPEED      I2C_SPEED_FAST

static int tcs37727_test(tcs37727_t *dev)
{
	uint8_t id;

	if (i2c_read_reg(dev->handle, dev->addr, TCS37727_ID, &id) != 1)
		return -1;

	if (id != TCS37727_ID_VALUE)
		return -1;

	return 0;
}

int tcs37727_init(tcs37727_t *dev, i2c_rtos_handle_t *handle, uint8_t address,
		  int atime_us)
{
	/* write device descriptor */
	dev->handle = handle;
	dev->addr = address;
	dev->initialized = false;

	if (tcs37727_test(dev))
		return -2;

	if (i2c_write_reg(dev->handle, dev->addr, TCS37727_CONTROL,
			  TCS37727_CONTROL_AGAIN_4) != 1)
		return -3;
	dev->again = 4;

	if (i2c_write_reg(dev->handle, dev->addr, TCS37727_ATIME,
			  TCS37727_ATIME_TO_REG(atime_us)) != 1)
		return -3;
	dev->atime_us = atime_us;

	dev->initialized = true;

	return 0;
}

int tcs37727_set_rgbc_active(tcs37727_t *dev)
{
	uint8_t reg;

	if (dev->initialized == false)
		return -1;

	if (i2c_read_regs(dev->handle, dev->addr, TCS37727_ENABLE,
			  &reg, 1) != 1)
		return -1;

	reg |= (TCS37727_ENABLE_AEN | TCS37727_ENABLE_PON);

	if (i2c_write_reg(dev->handle, dev->addr, TCS37727_ENABLE, reg) != 1)
		return -1;

	return 0;
}

int tcs37727_set_rgbc_standby(tcs37727_t *dev)
{
	uint8_t reg;

	if (dev->initialized == false)
		return -1;

	if (i2c_read_regs(dev->handle, dev->addr, TCS37727_ENABLE,
			  &reg, 1) != 1)
		return -1;

	reg &= ~TCS37727_ENABLE_AEN;
	if (!(reg & TCS37727_ENABLE_PEN))
		reg &= ~TCS37727_ENABLE_PON;

	if (i2c_write_reg(dev->handle, dev->addr, TCS37727_ENABLE, reg) != 1)
		return -1;

	return 0;
}

static uint8_t tcs37727_trim_gain(tcs37727_t *dev, int rawc)
{
	uint8_t reg_again = 0;
	int val_again = dev->again;

	if (dev->initialized == false)
		return -1;

	if (rawc < TCS37727_AG_THRESHOLD_LOW) {
		switch (val_again) {
		case 1:
			reg_again = TCS37727_CONTROL_AGAIN_4;
			val_again = 4;
			break;

		case 4:
			reg_again = TCS37727_CONTROL_AGAIN_16;
			val_again = 16;
			break;

		case 16:
			reg_again = TCS37727_CONTROL_AGAIN_60;
			val_again = 60;
			break;

		case 60:
		default:
			return -1;
		}
	}else if (rawc > TCS37727_AG_THRESHOLD_HIGH) {
		switch (val_again) {
		case 60:
			reg_again = TCS37727_CONTROL_AGAIN_16;
			val_again = 16;
			break;

		case 16:
			reg_again = TCS37727_CONTROL_AGAIN_4;
			val_again = 4;
			break;

		case 4:
			reg_again = TCS37727_CONTROL_AGAIN_1;
			val_again = 1;
			break;

		case 1:
		default:
			return -1;
		}
	}else
		return 0;

	uint8_t reg = 0;
	if (i2c_read_reg(dev->handle, dev->addr, TCS37727_CONTROL,
			 &reg) != 1)
		return -2;
	reg &= ~TCS37727_CONTROL_AGAIN_MASK;
	reg |= reg_again;
	if (i2c_write_reg(dev->handle, dev->addr, TCS37727_CONTROL, reg) != 1)
		return -2;
	dev->again = val_again;

	return 0;
}

int tcs37727_read(tcs37727_t *dev, tcs37727_data_t *data)
{
	uint8_t buf[8];

	if (dev->initialized == false)
		return -1;


	if (i2c_read_regs(dev->handle, dev->addr,
			  (TCS37727_INC_TRANS | TCS37727_CDATA), buf, 8) != 8)
		return -1;


	int32_t tmpc = ((uint16_t)buf[1] << 8) | buf[0];
	int32_t tmpr = ((uint16_t)buf[3] << 8) | buf[2];
	int32_t tmpg = ((uint16_t)buf[5] << 8) | buf[4];
	int32_t tmpb = ((uint16_t)buf[7] << 8) | buf[6];
	/*DEBUG("rawr: %"PRIi32" rawg %"PRIi32" rawb %"PRIi32" rawc %"PRIi32"\n",
		tmpr, tmpg, tmpb, tmpc);*/

	/* Remove IR component as described in the DN40.  */
	int32_t ir = (tmpr + tmpg + tmpb - tmpc) >> 1;
	tmpr -= ir;
	tmpg -= ir;
	tmpb -= ir;

	/* Color temperature calculation as described in the DN40. */
	int32_t ct = (CT_COEF_IF * tmpb) / tmpr + CT_OFFSET_IF;

	/* Lux calculation as described in the DN40.  */
	int32_t gi = R_COEF_IF * tmpr + G_COEF_IF * tmpg + B_COEF_IF * tmpb;
	/* TODO: add Glass Attenuation Factor GA compensation */
	int32_t cpl = (dev->atime_us * dev->again) / DGF_IF;
	int32_t lux = gi / cpl;

	/* Autogain */
	tcs37727_trim_gain(dev, tmpc);

	data->red = (tmpr < 0) ? 0 : (tmpr * 1000) / cpl;
	data->green = (tmpg < 0) ? 0 : (tmpg * 1000) / cpl;
	data->blue = (tmpb < 0) ? 0 : (tmpb * 1000) / cpl;
	data->clear = (tmpb < 0) ? 0 : (tmpc * 1000) / cpl;
	data->lux = (lux < 0) ? 0 : lux;
	data->ct = (ct < 0) ? 0 : ct;

	return 0;
}
