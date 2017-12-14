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

#ifndef _PERIPH_H
#define _PERIPH_H

/* PORT/GPIO/PIN defines for peripheral drivers */
#define RESET_PORT			PORTA
#define RESET_GPIO_DEV			GPIOA
#define RESET_FRAMEWORK_GPIO		gpioPort_A_c
#define RESET_PIN			2u

#define HDC1010_INT_PORT		PORTC
#define HDC1010_INT_GPIO_DEV		GPIOC
#define HDC1010_INT_FRAMEWORK_GPIO	gpioPort_C_c
#define HDC1010_INT_PIN			1u

#define TCS37727_INT_PORT		PORTC
#define TCS37727_INT_GPIO_DEV		GPIOC
#define TCS37727_INT_FRAMEWORK_GPIO	gpioPort_C_c
#define TCS37727_INT_PIN		4u

#define A_BUTTON_PORT			PORTC
#define A_BUTTON_GPIO_DEV		GPIOC
#define A_BUTTON_FRAMEWORK_GPIO		gpioPort_C_c
#define A_BUTTON_PIN			5u

#define SSD1673_DP_BUSY_PORT		PORTC
#define SSD1673_DP_BUSY_GPIO_DEV	GPIOC
#define SSD1673_DP_BUSY_FRAMEWORK_GPIO	gpioPort_C_c
#define SSD1673_DP_BUSY_PIN		16u
#define SSD1673_DP_RESET_PORT		PORTC
#define SSD1673_DP_RESET_GPIO_DEV	GPIOC
#define SSD1673_DP_RESET_FRAMEWORK_GPIO	gpioPort_C_c
#define SSD1673_DP_RESET_PIN		17u
#define SSD1673_DP_DC_PORT		PORTC
#define SSD1673_DP_DC_GPIO_DEV		GPIOC
#define SSD1673_DP_DC_FRAMEWORK_GPIO	gpioPort_C_c
#define SSD1673_DP_DC_PIN		18u

#define DSPI_MASTER_CLK_FREQ		CLOCK_GetFreq((DSPI1_CLK_SRC))
#define SSD1673_DSPI_MASTER_IRQN	(SPI1_IRQn)
#define SSD1673_DSPI_MASTER_BASEADDR	((SPI_Type *)(SPI1_BASE))

#define SSD1673_SPI_PORT		PORTA
#define SSD1673_SPI_SCK_PIN		16u
#define SSD1673_SPI_MOSI_PIN		17u
#define SSD1673_SPI_MISO_PIN		18u
#define SSD1673_SPI_PCS0_PIN		19u

#define V_PERIPH_PORT			PORTC
#define V_PERIPH_GPIO_DEV		GPIOC
#define V_PERIPH__FRAMEWORK_GPIO	gpioPort_C_c
#define V_PERIPH_PIN			19u

#define CCS811_NWAKE_PORT		PORTB
#define CCS811_NWAKE_GPIO_DEV		GPIOB
#define CCS811_NWAKE_FRAMEWORK_GPIO	gpioPort_B_c
#define CCS811_NWAKE_PIN		2u
#define CCS811_INT_PORT			PORTB
#define CCS811_INT_GPIO_DEV		GPIOB
#define CCS811_INT_FRAMEWORK_GPIO	gpioPort_B_c
/* CCS811 INT pin is default disconnected from GPIO PTB3 */
#define CCS811_INT_PIN			3u

#define YELLOW_LED_PORT			PORTB
#define YELLOW_LED_GPIO_DEV		GPIOB
#define YELLOW_LED_FRAMEWORK_GPIO	gpioPort_B_c
#define YELLOW_LED_PIN			3u

#define RGB_LED_PORT			PORTB
#define RGB_LED_GPIO_DEV		GPIOB
#define RGB_LED_FRAMEWORK_GPIO		gpioPort_B_c
#define RGB_LED_R_PIN			1u
#define RGB_LED_G_PIN			0u
#define RGB_LED_B_PIN			18u

#define I2C_PORT			PORTC
#define I2C_SCL_PIN			2u
#define I2C_SDA_PIN			3u

#define UART_PORT			PORTC
#define UART_TX_PIN			7u
#define UART_RX_PIN			6u

#define DW1000_SPI_CLK_SRC		CLOCK_GetFreq((DSPI1_CLK_SRC))
#define DW1000_DSPI_MASTER_IRQN		(SPI1_IRQn)
#define DW1000_DSPI_MASTER_BASEADDR	((SPI_Type *)(SPI1_BASE))

#define DW1000_INT_PORT			PORTC
#define DW1000_INT_GPIO_DEV		GPIOC
#define DW1000_INT_FRAMEWORK_GPIO	gpioPort_C_c
#define DW1000_INT_PIN			1u

#define DW1000_RESET_PORT		PORTC
#define DW1000_RESET_GPIO_DEV		GPIOC
#define DW1000_RESET_FRAMEWORK_GPIO	gpioPort_C_c
#define DW1000_RESET_PIN		2

#define DW1000_WAKEUP_PORT		PORTC
#define DW1000_WAKEUP_GPIO_DEV		GPIOC
#define DW1000_WAKEUP_FRAMEWORK_GPIO	gpioPort_C_c
#define DW1000_WAKEUP_PIN		3
#endif
