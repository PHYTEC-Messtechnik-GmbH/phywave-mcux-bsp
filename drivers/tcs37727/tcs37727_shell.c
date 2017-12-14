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
#include "tcs37727.h"

#ifndef TCS37727_ADDR
#define TCS37727_ADDR		0x29
#endif

static tcs37727_t tcs37727_dev;
static tcs37727_data_t tcs37727_data;
static i2c_rtos_handle_t *tcs37727_i2c_handle;

typedef struct {
	char* name;
	int8_t (*cmd)(uint8_t argc, char * argv[]);
} shell_cmd_table;

gpioInputPinConfig_t tcs37727_irq_pin_cfg = {
	.gpioPort = TCS37727_INT_FRAMEWORK_GPIO,
	.gpioPin = TCS37727_INT_PIN,
	.pullSelect = pinPull_Up_c,
	.interruptSelect = pinInt_FallingEdge_c
};

const char tcs37727_cmd_help[] = "\r\n"
			 "tcs37727 get\r\n"
			 "tcs37727 init\r\n"
			 "tcs37727 resume\r\n"
			 "tcs37727 standby\r\n";

static int8_t shell_tcs37727_cmd_get(uint8_t argc, char * argv[])
{
	if (argc != 0)
		return CMD_RET_USAGE;

	tcs37727_read(&tcs37727_dev, &tcs37727_data);
	shell_write("\n\r--> tcs37727 values: R: ");
	shell_writeDec(tcs37727_data.red);
	shell_write(" G: ");
	shell_writeDec(tcs37727_data.green);
	shell_write(" B: ");
	shell_writeDec(tcs37727_data.blue);
	shell_write(" C: ");
	shell_writeDec(tcs37727_data.clear);
	shell_write(" CT: ");
	shell_writeDec(tcs37727_data.ct);
	shell_write(" Lux: ");
	shell_writeDec(tcs37727_data.lux);
	shell_write(" AGAIN: ");
	shell_writeDec(tcs37727_dev.again);
	shell_write(" ATIME: ");
	shell_writeDec(tcs37727_dev.atime_us);

	PRINTF("CT: %d Lux: %d AGAIN: %d ATIME: %d\r\n",
	       tcs37727_data.ct,
	       tcs37727_data.lux,
	       tcs37727_dev.again,
	       tcs37727_dev.atime_us);

	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

static int8_t shell_tcs37727_cmd_init(uint8_t argc, char * argv[])
{
	if (argc != 0)
		return CMD_RET_USAGE;

	if (tcs37727_init(&tcs37727_dev, tcs37727_i2c_handle, TCS37727_ADDR,
			  TCS37727_ATIME_DEFAULT) != 0) {
		shell_write("--> tcs37727 initializing failed\r\n");
	} else {
		if (tcs37727_set_rgbc_active(&tcs37727_dev) != 0) {
			shell_write("--> Set tcs37727 active failed\r\n");
		} else {
			shell_write("--> Initialized tcs37727\n\r");
		}
	}

	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

static int8_t shell_tcs37727_cmd_resume(uint8_t argc, char * argv[])
{
	if (argc != 0)
		return CMD_RET_USAGE;

	if (tcs37727_set_rgbc_active(&tcs37727_dev) != 0) {
		shell_write("--> Set tcs37727 active failed\r\n");
	} else {
		shell_write("--> tcs37727 is active now\n\r");
	}

	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

static int8_t shell_tcs37727_cmd_standby(uint8_t argc, char * argv[])
{
	if (argc != 0)
		return CMD_RET_USAGE;

	if (tcs37727_set_rgbc_standby(&tcs37727_dev) != 0) {
		shell_write("--> set tcs37727 standby failed\r\n");
	} else {
		shell_write("--> standby tcs37727\n\r");
	}

	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

#define SHELL_TCS37727_NUMOF_CMD			4

const shell_cmd_table shell_tcs37727_table[SHELL_TCS37727_NUMOF_CMD] =
{
	{"get", shell_tcs37727_cmd_get},
	{"init", shell_tcs37727_cmd_init},
	{"resume", shell_tcs37727_cmd_resume},
	{"standby", shell_tcs37727_cmd_standby},
};

int8_t shell_tcs37727_cmd(uint8_t argc, char * argv[])
{
	uint8_t i;

	if (argc < 2) {
		return CMD_RET_USAGE;
	}

	for (i = 0; i < SHELL_TCS37727_NUMOF_CMD; i++) {
		if (!strcmp((char*)argv[1], shell_tcs37727_table[i].name) ) {
			return shell_tcs37727_table[i].cmd(argc - 2, (char**)(&argv[2]));
		}
	}

	return CMD_RET_USAGE;
}

const cmd_tbl_t tcs37727_cmd =
{
	.name		= "tcs37727",
	.maxargs	= 3,
	.repeatable	= 1,
	.cmd		= shell_tcs37727_cmd,
	.usage		= (char*)tcs37727_cmd_help,
	.help		= "Contains commands for advertising, scanning, " \
			  "connecting, pairing or disconnecting.",
};

int tcs37727_shell_init(i2c_rtos_handle_t *i2c_handle)
{
	tcs37727_i2c_handle = i2c_handle;
	if (tcs37727_i2c_handle) {
		shell_register_function((cmd_tbl_t*)&tcs37727_cmd);
		return 0;
	}

	return -1;
}
