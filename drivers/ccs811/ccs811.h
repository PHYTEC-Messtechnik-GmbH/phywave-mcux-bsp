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


#ifndef CCS811_H
#define CCS811_H

#include <stdint.h>
#include <stdbool.h>
#include "i2c_wrapper.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef CCS811_I2C_ADDRESS
#define CCS811_I2C_ADDRESS	0x5a /**< Default Device Address */
#endif

#ifndef CCS811_MEASURE_MODE
#define CCS811_MEASURE_MODE	CCS811_MEAS_MODE_DRIVE_MODE_1S
#endif

/**
 * @brief Device descriptor for CCS811 sensors.
 */
typedef struct {
	i2c_rtos_handle_t *handle; /**< I2C device handle */
	uint8_t addr;              /**< the sensor's slave address */
	bool initialized;          /**< true if sensor is initialized */
} ccs811_t;

/**
 * @brief CCS811 sensor test.
 * This function looks for Manufacturer ID of the CCS811 sensor.
 *
 * @param[in]  dev          device descriptor of sensor
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int ccs811_test(ccs811_t *dev);

/**
 * @brief Initialise the CCS811 sensor driver.
 *
 * @param[out] dev          device descriptor of sensor to initialize
 * @param[in]  handle       I2C handle of the bus the sensor is connected to
 * @param[in]  address      sensor's I2C slave address
 *
 * @return                  0 on success
 * @return                  -1 if initialization of I2C bus failed
 * @return                  -2 if sensor test failed
 * @return                  -3 if sensor configuration failed
 */
int ccs811_init(ccs811_t *dev, i2c_rtos_handle_t *handle, uint8_t address);

/**
 * @brief Check if the measurement is ready
 *
 * @param[in]  dev          device descriptor of sensor
 *
 * @return                  0 on data is ready
 * @return                  -1 on error
 */
int ccs811_isready(ccs811_t *dev);

/**
 * @brief Get CCS811 error identifier.
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] error_code   pointer of the variable to store the error code
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int ccs811_get_error_id(ccs811_t *dev, uint8_t *error_code);

/**
 * @brief Resume the sensor.
 *
 * @param[in]  dev          device descriptor of sensor
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int ccs811_resume(ccs811_t *dev);

/**
 * @brief Suspend the sensor.
 *
 * @param[in]  dev          device descriptor of sensor
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int ccs811_suspend(ccs811_t *dev);

/**
 * @brief Read sensor's data.
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] eco2         pointer to store the eC02 value
 * @param[out] tvoc         pointer to store the TVOC value
 * @param[out] status       pointer to store the status value
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int ccs811_read(ccs811_t *dev, uint16_t *eco2, uint16_t *tvoc, uint8_t *status);

#ifdef __cplusplus
}
#endif

#endif
/** @} */
