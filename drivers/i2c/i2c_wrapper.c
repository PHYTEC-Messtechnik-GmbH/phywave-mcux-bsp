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

#include "i2c_wrapper.h"
#include "pin_mux.h"
#include "Panic.h"
#include "semphr.h"
#include "fsl_os_abstraction.h"
#include "fsl_i2c_freertos.h"
#include "fsl_i2c.h"

static bool i2c_initialized = false;
static i2c_rtos_handle_t handle;
static SemaphoreHandle_t xMutex;

i2c_rtos_handle_t *i2c_init_master(I2C_Type* dev, uint32_t clk_src,
				   uint32_t baudrate)
{
	int status = 0;
	i2c_master_config_t masterConfig;
	uint32_t sourceClock = 0;

	if (i2c_initialized == true) {
		return 0;
	}

	xMutex = xSemaphoreCreateMutex();

	I2C_MasterGetDefaultConfig(&masterConfig);
	masterConfig.enableMaster = true;
	masterConfig.baudRate_Bps = baudrate;
	masterConfig.glitchFilterWidth = 0;
	sourceClock = CLOCK_GetFreq(clk_src);
	status = I2C_RTOS_Init(&handle, dev, &masterConfig, sourceClock);
	if (status == kStatus_Success) {
		i2c_initialized = true;
	} else {
		return NULL;
	}

	return &handle;
}

int i2c_read_reg(i2c_rtos_handle_t *handle, uint8_t address, uint8_t reg,
		 uint8_t *data)
{
	i2c_master_transfer_t transfer;

	memset(&transfer, 0, sizeof(transfer));
	transfer.slaveAddress = address;
	transfer.direction = kI2C_Read;
	transfer.subaddress = reg;
	transfer.subaddressSize = 1;
	transfer.data = (uint8_t*)data;
	transfer.dataSize = 1;
	transfer.flags = kI2C_TransferDefaultFlag;

        xSemaphoreTake(xMutex, portMAX_DELAY);
	I2C_RTOS_Transfer(handle, &transfer);
        xSemaphoreGive(xMutex);

	return 1;
}

int i2c_read_regs(i2c_rtos_handle_t *handle, uint8_t address, uint8_t reg,
		  uint8_t *data, int length)
{
	i2c_master_transfer_t transfer;

	memset(&transfer, 0, sizeof(transfer));
	transfer.slaveAddress = address;
	transfer.direction = kI2C_Read;
	transfer.subaddress = reg;
	transfer.subaddressSize = 1;
	transfer.data = (uint8_t*)data;
	transfer.dataSize = length;
	transfer.flags = kI2C_TransferDefaultFlag;

        xSemaphoreTake(xMutex, portMAX_DELAY);
	I2C_RTOS_Transfer(handle, &transfer);
        xSemaphoreGive(xMutex);

	return length;
}

int i2c_write_reg(i2c_rtos_handle_t *handle, uint8_t address, uint8_t reg,
		  uint8_t data)
{
	i2c_master_transfer_t transfer;

	memset(&transfer, 0, sizeof(transfer));
	transfer.slaveAddress = address;
	transfer.direction = kI2C_Write;
	transfer.subaddress = reg;
	transfer.subaddressSize = 1;
	transfer.data = (uint8_t*)&data;
	transfer.dataSize = 1;
	transfer.flags = kI2C_TransferDefaultFlag;

        xSemaphoreTake(xMutex, portMAX_DELAY);
	I2C_RTOS_Transfer(handle, &transfer);
        xSemaphoreGive(xMutex);

	return 1;
}

int i2c_write_regs(i2c_rtos_handle_t *handle, uint8_t address, uint8_t reg,
		   uint8_t *data, int length)
{
	i2c_master_transfer_t transfer;

	memset(&transfer, 0, sizeof(transfer));
	transfer.slaveAddress = address;
	transfer.direction = kI2C_Write;
	transfer.subaddress = reg;
	transfer.subaddressSize = 1;
	transfer.data = (uint8_t*)data;
	transfer.dataSize = length;
	transfer.flags = kI2C_TransferDefaultFlag;

        xSemaphoreTake(xMutex, portMAX_DELAY);
	I2C_RTOS_Transfer(handle, &transfer);
        xSemaphoreGive(xMutex);

	return length;
}

int i2c_write_byte(i2c_rtos_handle_t *handle, uint8_t address, uint8_t data)
{
	i2c_master_transfer_t transfer;

	memset(&transfer, 0, sizeof(transfer));
	transfer.slaveAddress = address;
	transfer.direction = kI2C_Write;
	transfer.subaddress = 0;
	transfer.subaddressSize = 0;
	transfer.data = (uint8_t*)&data;
	transfer.dataSize = 1;
	transfer.flags = kI2C_TransferDefaultFlag;

        xSemaphoreTake(xMutex, portMAX_DELAY);
	I2C_RTOS_Transfer(handle, &transfer);
        xSemaphoreGive(xMutex);

	return 1;
}

int i2c_read_bytes(i2c_rtos_handle_t *handle, uint8_t address, uint8_t *data,
		   int length)
{
	i2c_master_transfer_t transfer;

	memset(&transfer, 0, sizeof(transfer));
	transfer.slaveAddress = address;
	transfer.direction = kI2C_Read;
	transfer.subaddress = 0;
	transfer.subaddressSize = 0;
	transfer.data = (uint8_t*)data;
	transfer.dataSize = length;
	transfer.flags = kI2C_TransferDefaultFlag;

        xSemaphoreTake(xMutex, portMAX_DELAY);
	I2C_RTOS_Transfer(handle, &transfer);
        xSemaphoreGive(xMutex);

	return length;
}
