/*
 * Copyright (c) 2017 PHYTEC Messtechnik GmbH
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

#include "fsl_i2c.h"
#include "fsl_i2c_freertos.h"
#include "fsl_gpio.h"

#include "FreeRTOS.h"
#include "task.h"

#include "i2c_wrapper.h"
#include "ccs811.h"
#include "periph.h"

/**
 * Register Map
 */
#define CCS811_STATUS			0x00
#define CCS811_MEAS_MODE		0x01
#define CCS811_ALG_RESULT_DATA		0x02
#define CCS811_RAW_DATA			0x03
#define CCS811_ENV_DATA			0x05
#define CCS811_NTC			0x06
#define CCS811_THRESHOLDS		0x10
#define CCS811_BASELINE			0x11
#define CCS811_HW_ID			0x20
#define CCS811_HW_VERSION		0x21
#define CCS811_FW_BOOT_VERSION		0x23
#define CCS811_FW_APP_VERSION		0x24
#define CCS811_ERROR_ID			0xe0
#define CCS811_APP_ERASE		0xf1
#define CCS811_APP_DATA			0xf2
#define CCS811_APP_VERIFY		0xf3
#define CCS811_APP_START		0xf4
#define CCS811_SW_RESET			0xff

/**
 * Measure Mode Register
 */
#define CCS811_MEAS_MODE_THRESH			0x04
#define CCS811_MEAS_MODE_INTERRUPT		0x08
#define CCS811_MEAS_MODE_DRIVE_MODE(mode)	((mode & 0x7) << 4)
#define CCS811_MEAS_MODE_DRIVE_MODE_IDLE	0
#define CCS811_MEAS_MODE_DRIVE_MODE_1S		1
#define CCS811_MEAS_MODE_DRIVE_MODE_10S		2
#define CCS811_MEAS_MODE_DRIVE_MODE_60S		3
#define CCS811_MEAS_MODE_DRIVE_MODE_250MS	4

/**
 * Software Reset Key
 */
#define CCS811_SW_RESET_KEY		{0x11, 0xe5, 0x72, 0x8a}

/**
 * Status Register
 */
#define CCS811_STATUS_FW_MODE		0x80
#define CCS811_STATUS_APP_VALID		0x10
#define CCS811_STATUS_DATA_READY	0x08
#define CCS811_STATUS_ERROR		0x01

/**
 * Manufacturer and Device ID
 */
#define CCS811_HW_ID_VALUE		0x81
#define CCS811_HW_MASK			0x1f

static inline void ccs811_assert_nwake(void)
{
	GPIO_WritePinOutput(CCS811_NWAKE_GPIO_DEV, CCS811_NWAKE_PIN, 0);
}

static inline void ccs811_release_nwake(void)
{
	GPIO_WritePinOutput(CCS811_NWAKE_GPIO_DEV, CCS811_NWAKE_PIN, 1);
}

int ccs811_test(ccs811_t *dev)
{
	uint8_t id;

	if (i2c_read_regs(dev->handle, dev->addr, CCS811_HW_ID,
			  &id, 1) != 1) {
		return -1;
	}

	if (id != CCS811_HW_ID_VALUE) {
		return -1;
	}

	return 0;
}

int ccs811_init(ccs811_t *dev, i2c_rtos_handle_t *handle, uint8_t address)
{
	/* write device descriptor */
	dev->handle = handle;
	dev->addr = address;
	dev->initialized = false;
	uint8_t reset_key[4] = CCS811_SW_RESET_KEY;
	uint8_t status;

	ccs811_assert_nwake();

	if (ccs811_test(dev)) {
		goto ccs811_init_error;
	}

	/* reset the sensor */
	if (i2c_write_regs(dev->handle, dev->addr, CCS811_SW_RESET,
			   reset_key, 4) != 4) {
		goto ccs811_init_error;
	}

	vTaskDelay(10);

	/* check if valid firmware is present */
	if (i2c_read_regs(dev->handle, dev->addr, CCS811_STATUS,
			  &status, 1) != 1) {
		goto ccs811_init_error;
	}

	if (!(status & CCS811_STATUS_APP_VALID)) {
		goto ccs811_init_error;
	}

	/* start application mode */
	i2c_write_byte(dev->handle, dev->addr, CCS811_APP_START);

	/* check if the sensor is booted in the applicaiton mode */
	if (i2c_read_regs(dev->handle, dev->addr, CCS811_STATUS,
			  &status, 1) != 1) {
		goto ccs811_init_error;
	}

	if (!(status & (CCS811_STATUS_FW_MODE | CCS811_STATUS_APP_VALID))) {
		goto ccs811_init_error;
	}

	/* set default drive mode */
	uint8_t tmp = CCS811_MEAS_MODE_DRIVE_MODE(CCS811_MEASURE_MODE);
	if (i2c_write_regs(dev->handle, dev->addr, CCS811_MEAS_MODE,
			   &tmp, 1) != 1) {
		goto ccs811_init_error;
	}

	if (i2c_read_regs(dev->handle, dev->addr, CCS811_STATUS,
			  &status, 1) != 1) {
		goto ccs811_init_error;
	}

	if (status & (CCS811_STATUS_ERROR)) {
		goto ccs811_init_error;
	}

	dev->initialized = true;
	ccs811_release_nwake();

	return 0;

ccs811_init_error:
	ccs811_release_nwake();
	return -1;
}

int ccs811_isready(ccs811_t *dev)
{
	uint8_t status;

	if (dev->initialized == false)
		return -1;

	ccs811_assert_nwake();
	int tmp = i2c_read_reg(dev->handle, dev->addr, CCS811_STATUS,
			       &status);
	ccs811_release_nwake();
	if (tmp != 1) {
		return -1;
	}

	return !(status & CCS811_STATUS_DATA_READY);
}

int ccs811_resume(ccs811_t *dev)
{
	if (dev->initialized == false)
		return -1;

	ccs811_assert_nwake();
	uint8_t mode = CCS811_MEAS_MODE_DRIVE_MODE(CCS811_MEASURE_MODE);
	int tmp = i2c_write_regs(dev->handle, dev->addr, CCS811_MEAS_MODE,
				 &mode, 1);
	ccs811_release_nwake();

	if (tmp != 1) {
		return -1;
	}

	return 0;
}

int ccs811_suspend(ccs811_t *dev)
{
	if (dev->initialized == false)
		return -1;

	ccs811_assert_nwake();
	uint8_t mode = CCS811_MEAS_MODE_DRIVE_MODE(
				CCS811_MEAS_MODE_DRIVE_MODE_IDLE);
	int tmp = i2c_write_regs(dev->handle, dev->addr, CCS811_MEAS_MODE,
				 &mode, 1);
	ccs811_release_nwake();

	if (tmp != 1) {
		return -1;
	}

	return 0;
}

int ccs811_get_error_id(ccs811_t *dev, uint8_t *error_code)
{
	ccs811_assert_nwake();
	int tmp = i2c_read_reg(dev->handle, dev->addr, CCS811_ERROR_ID,
			       error_code);
	ccs811_release_nwake();
	if (tmp != 1) {
		return -1;
	}

	return 0;
}

int ccs811_read(ccs811_t *dev, uint16_t *eco2, uint16_t *tvoc, uint8_t *status)
{
	uint8_t data[5];

	if (dev->initialized == false)
		return -1;

	ccs811_assert_nwake();
	int tmp = i2c_read_regs(dev->handle, dev->addr, CCS811_ALG_RESULT_DATA,
				data, sizeof(data));
	ccs811_release_nwake();

	if (tmp != sizeof(data)) {
		return -1;
	}

	if ((data[4] & CCS811_STATUS_ERROR) || !(data[4] & CCS811_STATUS_DATA_READY)) {
		*eco2 = 0;
		*tvoc = 0;
		*status = 0;
		return -1;
	}

	*eco2 = ((uint16_t)data[0] << 8) | data[1];
	*tvoc = ((uint16_t)data[2] << 8) | data[3];
	*status = data[4];

	return 0;
}
