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

#ifndef HDC1010_H
#define HDC1010_H

#include <stdint.h>
#include <stdbool.h>
#include "i2c_wrapper.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef HDC1010_I2C_ADDRESS
#define HDC1010_I2C_ADDRESS           0x43 /**< Default Device Address */
#endif

#ifndef HDC1010_CONVERSION_TIME
#define HDC1010_CONVERSION_TIME       26000 /**< Default Conversion Time */
#endif

/**
 * @brief Device descriptor for HDC1010 sensors.
 */
typedef struct {
	i2c_rtos_handle_t *handle; /**< I2C device the sensor is connected to */
	uint8_t addr;              /**< the sensor's slave address on the I2C bus */
	bool initialized;          /**< sensor status, true if sensor is initialized */
} hdc1010_t;

/**
 * @brief HDC1010 sensor test.
 * This function looks for Manufacturer ID of the HDC1010 sensor.
 *
 * @param[in]  dev          device descriptor of sensor
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int hdc1010_test(hdc1010_t *dev);

/**
 * @brief Initialise the HDC1010 sensor driver.
 * 14 bit resolution, heater off, temperature and humidity
 * are acquired in sequence.
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
int hdc1010_init(hdc1010_t *dev, i2c_rtos_handle_t *handle, uint8_t address);

/**
 * @brief Reset the HDC1010 sensor. After that sensor should be reinitialized.
 *
 * @param[out] dev          device descriptor of sensor to reset
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int hdc1010_reset(hdc1010_t *dev);

/**
 * @brief Trigger the measurements.
 * Conversion Time by 14 bit resolution is 6.50ms.
 *
 * @param[in]  dev          device descriptor of sensor
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int hdc1010_startmeasure(hdc1010_t *dev);

/**
 * @brief Read sensor's data.
 *
 * @param[in]  dev          device descriptor of sensor
 * @param[out] rawtemp      raw temperature value
 * @param[out] rawhum       raw humidity value
 *
 * @return                  0 on success
 * @return                  -1 on error
 */
int hdc1010_read(hdc1010_t *dev, uint16_t *rawtemp, uint16_t *rawhum);

/**
 * @brief Convert raw sensor values to temperature and humidity.
 *
 * @param[in]  rawtemp      raw temperature value
 * @param[in]  rawhum       raw humidity value
 * @param[out] temp         converted temperature*100
 * @param[out] hum          converted humidity*100
 */
void hdc1010_convert(uint16_t rawtemp, uint16_t rawhum,  int *temp, int *hum);

#ifdef __cplusplus
}
#endif

#endif
/** @} */
