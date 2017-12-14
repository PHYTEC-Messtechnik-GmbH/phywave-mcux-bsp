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

#include <stdlib.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_dspi.h"
#include "fsl_dspi_freertos.h"
#include "fsl_gpio.h"

#include "board.h"
#include "gpio_pins.h"
#include "pin_mux.h"
#include "clock_config.h"

#include "ssd1673.h"
#include "yafg.h"

#define MASTER_TASK_PRIORITY          (configMAX_PRIORITIES - 1)

static void task_epaper(void *params)
{
	uint8_t cnt = 0;
	char str[12];

	PRINTF("*** phyNODE ePaper test ***\r\n");

	yafg_print("=== THIS IS AN EXAMPLE ===", 1, 0, -1);
	yafg_print("abcdefghijklmnopqrstuvwxyz", 2, 0, -1);
	yafg_print("We can also print special", 3, 0, -2);
	yafg_print("characters such as:", 4, 0, -2);
	yafg_print("!\"#$%&'()*+,-./[\\]^_{|}~", 5, 0, -1);
	yafg_framebuffer_invert();
	yafg_framebuffer_reverse_bytes();


	if (ssd1673_init()) {
		PRINTF("EPD initialization failed!\r\n");
		vTaskSuspend(NULL);
	}

	while (true) {
		yafg_framebuffer_reverse_bytes();
		yafg_framebuffer_invert();
		sprintf(str, "counter: %3d", cnt++);
		yafg_print(str, 6, 0, -1);
		yafg_framebuffer_invert();
		yafg_framebuffer_reverse_bytes();

		GPIO_WritePinOutput(YELLOW_LED_GPIO_DEV, YELLOW_LED_PIN, 0);

		ssd1673_set_initial_update();
		ssd1673_write_ram(yafg_get_framebuffer()->data,
				sizeof(yafg_get_framebuffer()->data));
		ssd1673_do_initial_update();

		GPIO_WritePinOutput(YELLOW_LED_GPIO_DEV, YELLOW_LED_PIN, 1);

		vTaskDelay(10000);
	}

	vTaskSuspend(NULL);
}

int main(void)
{
	hardware_init();
	BOARD_InitDebugConsole();
	BOARD_InitGPIO();
	BOARD_InitPins();
	BOARD_InitSPI();
	BOARD_InitLPUART();

	if (xTaskCreate(task_epaper, "task_epaper",
			configMINIMAL_STACK_SIZE + 64, NULL,
			MASTER_TASK_PRIORITY, NULL) != pdPASS) {
		PRINTF("Failed to create master task!\r\n");
	}

	vTaskStartScheduler();
	while (42);
}
