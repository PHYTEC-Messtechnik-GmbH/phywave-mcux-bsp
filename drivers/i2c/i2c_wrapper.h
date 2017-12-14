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

#ifndef __I2C_MAPPER_H__
#define __I2C_MAPPER_H__

#include "EmbeddedTypes.h"
#include "board.h"
#include "fsl_os_abstraction.h"
#include "fsl_i2c.h"
#include "fsl_i2c_freertos.h"
#include "fsl_debug_console.h"

/**
 * @brief   Write one byte to a register at the I2C slave with the given
 *          address
 *
 * @param[in] dev           I2C peripheral device
 * @param[in] address       bus address of the target device
 * @param[in] reg           the register address on the targeted I2C device
 * @param[in] data          array with bytes to write to the target device
 *
 * @return                  1
 * @return                  -1 on undefined device given
 */
int i2c_write_reg(i2c_rtos_handle_t *handle, uint8_t address, uint8_t reg,
		  uint8_t data);
/**
 * @brief   Write multiple bytes to a register at the I2C slave with the given
 *          address
 *
 * @param[in] dev           I2C peripheral device
 * @param[in] address       bus address of the target device
 * @param[in] reg           the register address on the targeted I2C device
 * @param[in] data          array with bytes to write to the target device
 * @param[in] length        number of bytes to write to the target device
 *
 * @return                  the number of bytes that were written
 * @return                  -1 on undefined device given
 */
int i2c_write_regs(i2c_rtos_handle_t *handle, uint8_t address, uint8_t reg,
		   uint8_t *data, int length);

/**
 * @brief   Initialize an I2C device to run as bus master
 *
 * @param[in] dev           the device to initialize
 * @param[in] clk_src       master clock source
 * @param[in] baudrate      the selected bus speed
 *
 * @return                  pointer on successful initialization
 * @return                  NULL on error
 */
i2c_rtos_handle_t *i2c_init_master(I2C_Type* dev, uint32_t clk_src,
				   uint32_t baudrate);

/**
 * @brief   Read one byte from a register at the I2C slave with the given
 *          address
 *
 * @param[in]  dev          I2C peripheral device
 * @param[in]  address      bus address of the target device
 * @param[in]  reg          the register address on the targeted I2C device
 * @param[out] data         array holding the received bytes
 *
 * @return                  1
 * @return                  -1 on undefined device given
 */
int i2c_read_reg(i2c_rtos_handle_t *handle, uint8_t address, uint8_t reg,
		 uint8_t *data);

/**
 * @brief   Read multiple bytes from a register at the I2C slave with the given
 *          address
 *
 * @param[in]  dev          I2C peripheral device
 * @param[in]  address      bus address of the target device
 * @param[in]  reg          the register address on the targeted I2C device
 * @param[out] data         array holding the received bytes
 * @param[in]  length       the number of bytes to read into `data`
 *
 * @return                  the number of bytes that were read
 * @return                  -1 on undefined device given
 */
int i2c_read_regs(i2c_rtos_handle_t *handle, uint8_t address, uint8_t reg,
		  uint8_t *data, int length);

/**
 * @brief   Write one byte to an I2C device with the given address
 *
 * @param[in] dev           I2C peripheral device
 * @param[in] address       bus address of the target device
 * @param[in] data          byte to write to the device
 *
 * @return                  the number of bytes that were written
 * @return                  -1 on undefined device given
 */
int i2c_write_byte(i2c_rtos_handle_t *handle, uint8_t address, uint8_t data);

/**
 * @brief   Read multiple bytes from an I2C device with the given address
 *
 * @param[in]  dev          I2C peripheral device
 * @param[in]  address      bus address of the target device
 * @param[out] data         array holding the received bytes
 * @param[in]  length       the number of bytes to read into `data`
 *
 * @return                  the number of bytes that were read
 * @return                  -1 on undefined device given
 */
int i2c_read_bytes(i2c_rtos_handle_t *handle, uint8_t address, uint8_t *data,
		   int length);

#endif
