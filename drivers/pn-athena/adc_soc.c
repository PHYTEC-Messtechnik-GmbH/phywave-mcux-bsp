/*
 * Copyright (c) 2017 PHYTEC Messtechnik GmbH
 *
 * Portions of this file are derived from NXP MCUXpresso SDK for the
 * Kinetis KW41Z, examples/bluetooth/temperature_sensor/freertos/board.c
 *
 * Copyright (c) 2013-2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>

#include "fsl_clock.h"
#include "fsl_adc16.h"
#include "fsl_pmc.h"
#include "adc_soc.h"

#define ADCDAC_LDO_OVERRIDE           (1)
#define ADC16_INSTANCE                (0) /* ADC instance */
#define ADC16_CHN_GROUP               (0) /* ADC group configuration selection */
#define ADC16_BATLVL_CHN              (23) /* Potentiometer channel */
#define ADC16_TEMP_SENSOR_CHN         (26) /* ADC channel of the Temperature Sensor */
#define ADC16_BANDGAP_CHN             (27) /* ADC channel of BANDGAP Voltage reference */

#define MIN_VOLT_BUCK                 (180)
#define MAX_VOLT_BUCK                 (310)
#define FULL_BAT                      (100)
#define EMPTY_BAT                     (0)

#define ADCR_VDD                      (4095U) /* Maximum value when use 12b resolution */
#define V_BG                          (1000U) /* BANDGAP voltage in mV (trim to 1.0V) */
#define V_TEMP25                      (716U)  /* Typical converted value at 25 oC in mV */
#define M                             (1620U) /* Typical slope:uV/oC */
#define STANDARD_TEMP                 (25)

uint32_t offsetVdd = 0;
static uint32_t adcValue = 0;           /* ADC value */
static adc16_config_t adcUserConfig;    /* structure for user config */

void ADC16_CalibrateParams(void)
{
#if gDCDC_Enabled_d == 0
	ADC16_DoAutoCalibration(ADC0);
	ADC16_SetHardwareAverage(ADC0, kADC16_HardwareAverageCount16);
#endif

	pmc_bandgap_buffer_config_t pmcBandgapConfig =
	{
		.enable			= true, /*!< Enable bandgap buffer.                   */
	};

	// Enable BANDGAP reference voltage
	PMC_ConfigureBandgapBuffer(PMC, &pmcBandgapConfig);
}

/*!
 * @brief Reads the ADC value from the channel given as input
 *
 * This function measure the ADC channel given as input
 */
static uint16_t ADC16_ReadValue(uint32_t chnIdx, uint8_t diffMode)
{
	adc16_channel_config_t chnConfig;

	/* Configure the conversion channel */
	chnConfig.channelNumber     = chnIdx;
	chnConfig.enableDifferentialConversion = diffMode;
	chnConfig.enableInterruptOnConversionCompleted  = false;

	/* Software trigger the conversion */
	ADC16_SetChannelConfig(ADC0, ADC16_CHN_GROUP, &chnConfig);
	/* Wait for the conversion to be done */
	while (0U == (kADC16_ChannelConversionDoneFlag &
		      ADC16_GetChannelStatusFlags(ADC0, ADC16_CHN_GROUP))) ;

	/* Fetch the conversion value */
	adcValue =  ADC16_GetChannelConversionValue(ADC0, ADC16_CHN_GROUP);

	return adcValue;
}

/*!
 * @brief Gets the current voltage of the battery
 *
 * This function measure the ADC channel corresponding to the battery
 */

static inline uint32_t ADC16_BatLvl(void)
{
	adcValue = ADC16_ReadValue(ADC16_BATLVL_CHN, false);
	return adcValue;
}

/*!
 * @brief Gets the current bandgap voltage
 *
 * This function measure the ADC channel corresponding to the bandgap
 */

static inline uint32_t ADC16_BgLvl(void)
{
	adcValue = ADC16_ReadValue(ADC16_BANDGAP_CHN, false);
	return adcValue;
}

void BOARD_InitAdc(void)
{
#if (gDCDC_Enabled_d == 0) || ((gDCDC_Enabled_d == 1) && (APP_DCDC_MODE == gDCDC_Mode_Bypass_c))
	CLOCK_EnableClock(kCLOCK_Dcdc0);
	CLOCK_EnableClock(kCLOCK_Adc0);
	ADC16_GetDefaultConfig(&adcUserConfig);
	adcUserConfig.resolution = kADC16_ResolutionSE12Bit;
	adcUserConfig.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
	ADC16_Init(ADC0, &adcUserConfig);
	ADC16_CalibrateParams();
#endif
}

uint16_t BOARD_GetBatteryVoltage(void)
{
	uint16_t batVal, bandgapValue, batVolt, bgVolt = 100; /*cV*/

	bandgapValue = ADC16_BgLvl();
	batVal = ADC16_BatLvl() * DCDC_VBAT_ADC_DIVIDER;

	batVolt = bgVolt * batVal / bandgapValue;

	return batVolt;
}

uint8_t BOARD_GetBatteryLevel(void)
{
	uint16_t batVal, bandgapValue, batLvl, batVolt, bgVolt = 100; /*cV*/

	bandgapValue = ADC16_BgLvl();
	batVal = ADC16_BatLvl() * DCDC_VBAT_ADC_DIVIDER;

	batVolt = bgVolt * batVal / bandgapValue;

	batLvl = (batVolt - MIN_VOLT_BUCK) * (FULL_BAT - EMPTY_BAT) /
		 (MAX_VOLT_BUCK - MIN_VOLT_BUCK);
	return (batLvl <= 100) ? batLvl : 100;
}

int32_t BOARD_GetTemperature(void)
{
	uint16_t tempVal, bandgapValue, tempVolt, bgVolt = 100; /*cV*/
	uint32_t vdd, adcrTemp25, adcr100m;

	bandgapValue = ADC16_BgLvl();
	tempVal = ADC16_ReadValue(ADC16_TEMP_SENSOR_CHN, false);

	tempVolt = bgVolt * tempVal / bandgapValue;
	(void)tempVolt;

	/* Get VDD value measured in mV */
	/* VDD = (ADCR_VDD x V_BG) / ADCR_BG  */
	vdd = ADCR_VDD * V_BG / bandgapValue;
	/* Calibrate ADCR_TEMP25  */
	/* ADCR_TEMP25 = ADCR_VDD x V_TEMP25 / VDD  */
	adcrTemp25 = ADCR_VDD * V_TEMP25 / vdd;
	/* Calculate conversion value of 100mV. */
	/* ADCR_100M = ADCR_VDD x 100 / VDD */
	adcr100m = ADCR_VDD * 100 / vdd;

	/* Multiplied by 1000 because M in uM/oC */
	/* Temperature = 25 - (ADCR_T - ADCR_TEMP25) * 100*1000 / ADCR_100M*M */
	return (int32_t)(STANDARD_TEMP -
			 ((int32_t)adcValue - (int32_t)adcrTemp25) * 100000 /
			 (int32_t)(adcr100m * M));
}

