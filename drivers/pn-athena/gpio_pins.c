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

#include "GPIO_Adapter.h"
#include "gpio_pins.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "periph.h"
#include "pin_mux.h"

gpioInputPinConfig_t switchPins[] = {
	{
		.gpioPort = A_BUTTON_FRAMEWORK_GPIO,
		.gpioPin = A_BUTTON_PIN,
		.pullSelect = pinPull_Up_c,
		.interruptSelect = pinInt_FallingEdge_c
	},
};

gpioOutputPinConfig_t ledPins[] = {
	{
		.gpioPort = YELLOW_LED_FRAMEWORK_GPIO,
		.gpioPin = YELLOW_LED_PIN,
		.outputLogic = 0,
		.slewRate = pinSlewRate_Slow_c,
		.driveStrength = pinDriveStrength_Low_c
	},
};

typedef struct {
	PORT_Type *port_base;
	GPIO_Type *gpio_base;
	gpio_pin_config_t pin_cfg;
	uint8_t pin;
	uint32_t pcr;
	bool not_sleep;
} ya_gpio_config_t;

static ya_gpio_config_t ya_gpio_cfg[] = {
	{
		/* PTC19 */
		V_PERIPH_PORT,
		V_PERIPH_GPIO_DEV,
		{kGPIO_DigitalOutput, 0},
		V_PERIPH_PIN,
		0,
		false,
	},
	{
		/* PTB3 */
		YELLOW_LED_PORT,
		YELLOW_LED_GPIO_DEV,
		{kGPIO_DigitalOutput, 0},
		YELLOW_LED_PIN,
		0,
		false,
	},
	{
		/* PTB1 */
		RGB_LED_PORT,
		NULL,
		{0, 0},
		RGB_LED_R_PIN,
		0,
		false,
	},
	{
		/* PTB0 */
		RGB_LED_PORT,
		NULL,
		{0, 0},
		RGB_LED_G_PIN,
		0,
		false,
	},
	{
		/* PTB18 */
		RGB_LED_PORT,
		NULL,
		{0, 0},
		RGB_LED_B_PIN,
		0,
		false,
	},
	//{
	//	/* PTC1 */
	//	HDC1010_INT_PORT,
	//	HDC1010_INT_GPIO_DEV,
	//	{kGPIO_DigitalInput, 0},
	//	HDC1010_INT_PIN,
	//	0,
	//	false,
	//},
	{
		/* PTC4 */
		TCS37727_INT_PORT,
		TCS37727_INT_GPIO_DEV,
		{kGPIO_DigitalInput, 0},
		TCS37727_INT_PIN,
		0,
		false,
	},
	//{
	//	/* PTC5 */
	//	A_BUTTON_PORT,
	//	A_BUTTON_GPIO_DEV,
	//	{kGPIO_DigitalInput, 0},
	//	A_BUTTON_PIN,
	//	0,
	//	true,
	//},
	{
		/* PTB2 */
		CCS811_NWAKE_PORT,
		CCS811_NWAKE_GPIO_DEV,
		{kGPIO_DigitalOutput, 1},
		CCS811_NWAKE_PIN,
		0,
		true,
	},
	{
		/* PTC2 */
		I2C_PORT,
		NULL,
		{0, 0},
		I2C_SDA_PIN,
		0,
		false,
	},
	{
		/* PTC3 */
		I2C_PORT,
		NULL,
		{0, 0},
		I2C_SCL_PIN,
		0,
		false,
	},
	{
		/* PTC6 */
		UART_PORT,
		NULL,
		{0, 0},
		UART_RX_PIN,
		0,
		false,
	},
	{
		/* PTC7 */
		UART_PORT,
		NULL,
		{0, 0},
		UART_TX_PIN,
		0,
		false,
	},
	{
		SSD1673_DP_BUSY_PORT,
		SSD1673_DP_BUSY_GPIO_DEV,
		{kGPIO_DigitalInput, 0},
		SSD1673_DP_BUSY_PIN,
		0,
		true,
	},
	{
		SSD1673_DP_RESET_PORT,
		SSD1673_DP_RESET_GPIO_DEV,
		{kGPIO_DigitalOutput, 1},
		SSD1673_DP_RESET_PIN,
		0,
		true,
	},
	{
		SSD1673_DP_DC_PORT,
		SSD1673_DP_DC_GPIO_DEV,
		{kGPIO_DigitalOutput, 0},
		SSD1673_DP_DC_PIN,
		0,
		true,
	},
	{
		/* PTA16 */
		SSD1673_SPI_PORT,
		NULL,
		{0, 0},
		SSD1673_SPI_SCK_PIN,
		0,
		false,
	},
	{
		/* PTA17 */
		SSD1673_SPI_PORT,
		NULL,
		{0, 0},
		SSD1673_SPI_MOSI_PIN,
		0,
		false,
	},
	{
		/* PTA18 */
		SSD1673_SPI_PORT,
		NULL,
		{0, 0},
		SSD1673_SPI_MISO_PIN,
		0,
		false,
	},
	{
		/* PTA19 */
		SSD1673_SPI_PORT,
		NULL,
		{0, 0},
		SSD1673_SPI_PCS0_PIN,
		0,
		false,
	 },
};

void BOARD_InitGPIO(void)
{
	int numof_cfg = sizeof(ya_gpio_cfg) / sizeof(ya_gpio_config_t);

	for (int idx = 0; idx < numof_cfg; idx ++) {
		if (ya_gpio_cfg[idx].gpio_base == NULL) {
			continue;
		}
		GPIO_PinInit(ya_gpio_cfg[idx].gpio_base, ya_gpio_cfg[idx].pin,
			     &ya_gpio_cfg[idx].pin_cfg);
	}
}

void BOARD_SetPinsLowPower(bool_t lowpower)
{
	int numof_cfg = sizeof(ya_gpio_cfg) / sizeof(ya_gpio_config_t);

	if (lowpower) {
		for (int idx = 0; idx < numof_cfg; idx ++) {
			if (ya_gpio_cfg[idx].not_sleep) {
				continue;
			}
			ya_gpio_cfg[idx].pcr =
				ya_gpio_cfg[idx].port_base->PCR[ya_gpio_cfg[idx].pin];

			ya_gpio_cfg[idx].port_base->PCR[ya_gpio_cfg[idx].pin] =
				(ya_gpio_cfg[idx].pcr &
				~(PORT_PCR_MUX_MASK | PORT_PCR_ISF_MASK)) |
				PORT_PCR_MUX(kPORT_PinDisabledOrAnalog);
		}
	} else  {
		for (int idx = numof_cfg - 1; idx >= 0; idx --) {
			if (ya_gpio_cfg[idx].not_sleep) {
				continue;
			}
			ya_gpio_cfg[idx].port_base->PCR[ya_gpio_cfg[idx].pin] =
				ya_gpio_cfg[idx].pcr & ~(PORT_PCR_ISF_MASK) ;

		}
	}
}
