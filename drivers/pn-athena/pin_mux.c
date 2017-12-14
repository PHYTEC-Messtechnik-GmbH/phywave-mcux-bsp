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

#include "fsl_common.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "pin_mux.h"

void BOARD_InitButtons(void)
{
	/* a Button (PTC5) */
	PORT_SetPinMux(A_BUTTON_PORT, A_BUTTON_PIN, kPORT_MuxAsGpio);
}

void BOARD_InitLEDs(void)
{
	/* Yellow LED on PTB3, the GPIO can also be used for CCS811 IRQ */
	PORT_SetPinMux(YELLOW_LED_PORT, YELLOW_LED_PIN, kPORT_MuxAsGpio);

}

void BOARD_InitPins(void)
{
	CLOCK_EnableClock(kCLOCK_PortB);
	CLOCK_EnableClock(kCLOCK_PortC);

	/* PTC1 configured as GPIO, connected to HDC1010 DRDYn*/
	const port_pin_config_t ptc1_pin_config = {
		kPORT_PullUp,
		kPORT_SlowSlewRate,
		kPORT_PassiveFilterDisable,
		kPORT_LowDriveStrength,
		kPORT_MuxAsGpio,
	};
	PORT_SetPinConfig(HDC1010_INT_PORT, HDC1010_INT_PIN, &ptc1_pin_config);

	/* PTC4 configured as GPIO, connected to TCS37727 INT */
	const port_pin_config_t ptc4_pin_config = {
		kPORT_PullUp,
		kPORT_SlowSlewRate,
		kPORT_PassiveFilterDisable,
		kPORT_LowDriveStrength,
		kPORT_MuxAsGpio,
	};
	PORT_SetPinConfig(TCS37727_INT_PORT, TCS37727_INT_PIN, &ptc4_pin_config);

	/* PTB2 configured as GPIO, connected to CCS811 nWAKE */
	PORT_SetPinMux(CCS811_NWAKE_PORT, CCS811_NWAKE_PIN, kPORT_MuxAsGpio);

	/* EPD BUSY(9) */
	PORT_SetPinMux(SSD1673_DP_BUSY_PORT, SSD1673_DP_BUSY_PIN, kPORT_MuxAsGpio);
	/* EPD nRES(10) */
	PORT_SetPinMux(SSD1673_DP_RESET_PORT, SSD1673_DP_RESET_PIN, kPORT_MuxAsGpio);
	/* EPD nD/C(11) */
	PORT_SetPinMux(SSD1673_DP_DC_PORT, SSD1673_DP_DC_PIN, kPORT_MuxAsGpio);

	/* PTC19 configured as GPIO and should be used for V_PERIPH on/off */
	PORT_SetPinMux(V_PERIPH_PORT, V_PERIPH_PIN, kPORT_MuxAsGpio);

	BOARD_InitButtons();
	BOARD_InitLEDs();
}

void BOARD_InitRGB(void)
{
	CLOCK_EnableClock(kCLOCK_PortB);

	/* Red LED (PTB1), TMP0_CH2 */
	PORT_SetPinMux(RGB_LED_PORT, RGB_LED_R_PIN, kPORT_MuxAlt5);
	/* Green LED (PTB0), TMP0_CH1 */
	PORT_SetPinMux(RGB_LED_PORT, RGB_LED_G_PIN, kPORT_MuxAlt5);
	/* Blue LED (PTB18), TMP0_CH0 */
	PORT_SetPinMux(RGB_LED_PORT, RGB_LED_B_PIN, kPORT_MuxAlt5);
}

void BOARD_InitSPI(void)
{
	CLOCK_EnableClock(kCLOCK_PortA);
	CLOCK_EnableClock(kCLOCK_PortC);

	/* EPD or Arduino connector: SPI1_SCK, SPI1_SOUT, SPI1_SIN, SPI1_PCS0 */
	PORT_SetPinMux(SSD1673_SPI_PORT, SSD1673_SPI_SCK_PIN, kPORT_MuxAlt2);
	PORT_SetPinMux(SSD1673_SPI_PORT, SSD1673_SPI_MOSI_PIN, kPORT_MuxAlt2);
	PORT_SetPinMux(SSD1673_SPI_PORT, SSD1673_SPI_MISO_PIN, kPORT_MuxAlt2);
	PORT_SetPinMux(SSD1673_SPI_PORT, SSD1673_SPI_PCS0_PIN, kPORT_MuxAlt2);

	/* SPI0_SCK, SPI0_SOUT, SPI0_SIN, SPI0_PCS0 */
	/* SPI0 is disabled, because the Pins are used as GPIOs.
	PORT_SetPinMux(PORTC, PTC16_IDX, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTC, PTC17_IDX, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTC, PTC18_IDX, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTC, PTC19_IDX, kPORT_MuxAlt2);
	*/
}

void BOARD_InitLPUART(void)
{
	CLOCK_EnableClock(kCLOCK_PortC);

	/* UART0_RX, UART0_TX */
	PORT_SetPinMux(UART_PORT, UART_RX_PIN, kPORT_MuxAlt4);
	PORT_SetPinMux(UART_PORT, UART_TX_PIN, kPORT_MuxAlt4);

	/* LPUART0 Receive Data Source Select: LPUART_RX pin */
	SIM->SOPT5 &= ~(SIM_SOPT5_LPUART0RXSRC_MASK);

}

void BOARD_InitI2C(void)
{
	CLOCK_EnableClock(kCLOCK_PortC);

	/* PTC2, PTC3 configured as I2C1_SCL, I2C1_SDA */
	const port_pin_config_t ptc2_pin_d5_config = {
		kPORT_PullUp,
		kPORT_SlowSlewRate,
		kPORT_PassiveFilterDisable,
		kPORT_LowDriveStrength,
		kPORT_MuxAlt3,
	};
	PORT_SetPinConfig(I2C_PORT, I2C_SCL_PIN, &ptc2_pin_d5_config);
	const port_pin_config_t ptc3_pin_c5_config = {
		kPORT_PullUp,
		kPORT_SlowSlewRate,
		kPORT_PassiveFilterDisable,
		kPORT_LowDriveStrength,
		kPORT_MuxAlt3,
	};
	PORT_SetPinConfig(I2C_PORT, I2C_SDA_PIN, &ptc3_pin_c5_config);
}
