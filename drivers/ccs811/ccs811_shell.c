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
#include "ccs811.h"

static ccs811_t ccs811_dev;
static i2c_rtos_handle_t *ccs811_i2c_handle;

typedef struct {
	char* name;
	int8_t (*cmd)(uint8_t argc, char * argv[]);
} shell_cmd_table;

static int8_t shell_ccs811_cmd_get(uint8_t argc, char * argv[])
{
	uint16_t eco2, tvoc;
	uint8_t status;

	if (argc != 0)
		return CMD_RET_USAGE;

	if (ccs811_read(&ccs811_dev, &eco2, &tvoc, &status)) {
		shell_write("\n\r--> CCS811 get failed or not ready\r\n");
		goto ccs811_cmd_get_exit;
	}
	shell_write("\n\r--> ccs811 values: eCO2: ");
	shell_writeDec(eco2);
	shell_write(" TVOC: ");
	shell_writeDec(tvoc);
	shell_write(" status: ");
	shell_writeHex(&status, sizeof(status));
	shell_write("\n\r");

ccs811_cmd_get_exit:
	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

static int8_t shell_ccs811_cmd_init(uint8_t argc, char * argv[])
{
	if (argc != 0)
		return CMD_RET_USAGE;

	GPIO_WritePinOutput(CCS811_NWAKE_GPIO_DEV, CCS811_NWAKE_PIN, 0);
	if (ccs811_init(&ccs811_dev, ccs811_i2c_handle, CCS811_I2C_ADDRESS) != 0) {
		shell_write("\n\r--> CCS811 initializing failed\r\n");
	} else {
		shell_write("\n\r--> Initialized CCS811\n\r");
	}
	GPIO_WritePinOutput(CCS811_NWAKE_GPIO_DEV, CCS811_NWAKE_PIN, 1);

	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

static int8_t shell_ccs811_cmd_suspend(uint8_t argc, char * argv[])
{
	if (argc != 0)
		return CMD_RET_USAGE;

	if (ccs811_suspend(&ccs811_dev) != 0) {
		shell_write("\n\r--> CCS811 suspend failed\r\n");
	} else {
		shell_write("\n\r--> ok\n\r");
	}

	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

static int8_t shell_ccs811_cmd_resume(uint8_t argc, char * argv[])
{
	if (argc != 0)
		return CMD_RET_USAGE;

	if (ccs811_resume(&ccs811_dev) != 0) {
		shell_write("\n\r--> CCS811 resume failed\r\n");
	} else {
		shell_write("\n\r--> ok\n\r");
	}

	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

static int8_t shell_ccs811_cmd_error(uint8_t argc, char * argv[])
{
	uint8_t error_code;

	if (argc != 0)
		return CMD_RET_USAGE;

	if (ccs811_get_error_id(&ccs811_dev, &error_code) != 0) {
		shell_write("\n\r--> failed to read CCS811 error code");
	} else {
		shell_write("\n\r--> CCS811 error code 0x");
		shell_writeHexLe(&error_code, 1);
	}

	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

#define SHELL_CCS811_NUMOF_CMD			5

const shell_cmd_table shell_ccs811_table[SHELL_CCS811_NUMOF_CMD] =
{
	{"get", shell_ccs811_cmd_get},
	{"init", shell_ccs811_cmd_init},
	{"suspend", shell_ccs811_cmd_suspend},
	{"resume", shell_ccs811_cmd_resume},
	{"error", shell_ccs811_cmd_error},
};

const char ccs811_cmd_help[] = "\r\n"
			 "ccs811 get\r\n"
			 "ccs811 init\r\n"
			 "ccs811 suspend\r\n"
			 "ccs811 resume\r\n"
			 "ccs811 error\r\n";

int8_t shell_ccs811_cmd(uint8_t argc, char * argv[])
{
	uint8_t i;

	if (argc < 2) {
		return CMD_RET_USAGE;
	}

	for (i = 0; i < SHELL_CCS811_NUMOF_CMD; i++) {
		if (!strcmp((char*)argv[1], shell_ccs811_table[i].name) ) {
			return shell_ccs811_table[i].cmd(argc - 2, (char**)(&argv[2]));
		}
	}

	return CMD_RET_USAGE;
}

const cmd_tbl_t ccs811_cmd =
{
	.name		= "ccs811",
	.maxargs	= 3,
	.repeatable	= 1,
	.cmd		= shell_ccs811_cmd,
	.usage		= (char*)ccs811_cmd_help,
	.help		= "Contains commands for advertising, scanning, " \
			  "connecting, pairing or disconnecting.",
};

int ccs811_shell_init(i2c_rtos_handle_t *i2c_handle)
{
	ccs811_i2c_handle = i2c_handle;
	if (ccs811_i2c_handle) {
		shell_register_function((cmd_tbl_t*)&ccs811_cmd);
		return 0;
	}

	return -1;
}
