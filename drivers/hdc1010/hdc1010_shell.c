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

#include <string.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "fsl_i2c.h"
#include "fsl_i2c_freertos.h"
#include "fsl_gpio.h"
#include "semphr.h"
#include "fsl_os_abstraction.h"

#include "GPIO_Adapter.h"

#include "pin_mux.h"
#include "periph.h"

#include "shell.h"
#include "hdc1010.h"

#ifndef HDC1010_ADDR
#define HDC1010_ADDR		0x43
#endif

#define HDC1010_TIMEOUT_MS	(50/portTICK_PERIOD_MS)

static hdc1010_t hdc1010_dev;
static i2c_rtos_handle_t *hdc1010_i2c_handle;
static SemaphoreHandle_t xMutex;

typedef struct {
	char* name;
	int8_t (*cmd)(uint8_t argc, char * argv[]);
} shell_cmd_table;

gpioInputPinConfig_t hdc1010_irq_pin_cfg = {
	.gpioPort = HDC1010_INT_FRAMEWORK_GPIO,
	.gpioPin = HDC1010_INT_PIN,
	.pullSelect = pinPull_Up_c,
	.interruptSelect = pinInt_FallingEdge_c
};

const char hdc1010_cmd_help[] = "\r\n"
				"hdc1010 get\r\n"
				"hdc1010 init\r\n"
				"hdc1010 reset\r\n";

static void hdc1010_irq_cb(void)
{
	GpioClearPinIntFlag(&hdc1010_irq_pin_cfg);
	xSemaphoreGive(xMutex);
}

static int hdc1010_setup_interrupt_cb(void)
{
	GpioInputPinInit(&hdc1010_irq_pin_cfg, 1);
	GpioInstallIsr(hdc1010_irq_cb, gGpioIsrPrioLow_c, 0x80, &hdc1010_irq_pin_cfg);
	return 0;
}

static int8_t shell_hdc1010_cmd_get(uint8_t argc, char * argv[])
{
	uint16_t rawtemp, rawhum;
	int temp, hum;

	if (argc != 0)
		return CMD_RET_USAGE;

	if (hdc1010_startmeasure(&hdc1010_dev)) {
		shell_write("--> Measurement start failed.\r\n");
		goto hdc1010_cmd_get_exit;
	}

	if(xSemaphoreTake(xMutex, (TickType_t)HDC1010_TIMEOUT_MS)) {
		hdc1010_read(&hdc1010_dev, &rawtemp, &rawhum);
		hdc1010_convert(rawtemp, rawhum, &temp, &hum);

		shell_write("--> hdc1010 values: T: ");
		shell_writeDec(temp);
		shell_write(" HR: ");
		shell_writeDec(hum);
		shell_write("\n\r");
	} else {
		shell_write("--> hdc1010 timeout\n\r");
	}
	/*while(GPIO_ReadPinInput(BOARD_HDC1010_IRQ_GPIO, BOARD_HDC1010_IRQ_PIN));*/

hdc1010_cmd_get_exit:
	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

static int8_t shell_hdc1010_cmd_init(uint8_t argc, char * argv[])
{
	if (argc != 0)
		return CMD_RET_USAGE;

	if (hdc1010_init(&hdc1010_dev, hdc1010_i2c_handle, HDC1010_ADDR) != 0) {
		shell_write("--> HDC1010 initializing failed\r\n");
	} else {
		shell_write("--> Initialized HDC1010\n\r");
	}

	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

static int8_t shell_hdc1010_cmd_reset(uint8_t argc, char * argv[])
{
	if (argc != 0)
		return CMD_RET_USAGE;

	if (hdc1010_reset(&hdc1010_dev) != 0) {
		shell_write("--> HDC1010 reset failed\r\n");
	} else {
		shell_write("--> reset HDC1010\n\r");
	}

	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

#define SHELL_HDC1010_NUMOF_CMD			3

const shell_cmd_table shell_hdc1010_table[SHELL_HDC1010_NUMOF_CMD] =
{
	{"get", shell_hdc1010_cmd_get},
	{"init", shell_hdc1010_cmd_init},
	{"reset", shell_hdc1010_cmd_reset},
};

int8_t shell_hdc1010_cmd(uint8_t argc, char * argv[])
{
	uint8_t i;

	if (argc < 2) {
		return CMD_RET_USAGE;
	}

	for (i = 0; i < SHELL_HDC1010_NUMOF_CMD; i++) {
		if (!strcmp((char*)argv[1], shell_hdc1010_table[i].name) ) {
			return shell_hdc1010_table[i].cmd(argc - 2, (char**)(&argv[2]));
		}
	}

	return CMD_RET_USAGE;
}

const cmd_tbl_t hdc1010_cmd =
{
	.name		= "hdc1010",
	.maxargs	= 3,
	.repeatable	= 1,
	.cmd		= shell_hdc1010_cmd,
	.usage		= (char*)hdc1010_cmd_help,
	.help		= "Contains commands for advertising, scanning, " \
			  "connecting, pairing or disconnecting.",
};

int hdc1010_shell_init(i2c_rtos_handle_t *i2c_handle)
{
	hdc1010_i2c_handle = i2c_handle;
	hdc1010_setup_interrupt_cb();
	xMutex = xSemaphoreCreateBinary();
	if (hdc1010_i2c_handle) {
		shell_register_function((cmd_tbl_t*)&hdc1010_cmd);
		return 0;
	}

	return -1;
}
