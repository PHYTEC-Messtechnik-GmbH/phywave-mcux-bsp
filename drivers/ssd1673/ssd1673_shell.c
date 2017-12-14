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

#include "FreeRTOS.h"
#include "fsl_dspi.h"
#include "fsl_dspi_freertos.h"

#include "GPIO_Adapter.h"

#include "pin_mux.h"
#include "periph.h"

#include "shell.h"
#include "ssd1673.h"
#include "ssd1673_shell.h"

#include "yafg.h"

typedef struct {
	char *name;
	int8_t (*cmd)(uint8_t argc, char *argv[]);
} shell_cmd_table;

static int8_t shell_ssd1673_cmd_init(uint8_t argc, char * argv[])
{
	if (argc != 0)
		return CMD_RET_USAGE;

	if (ssd1673_init()) {
		shell_write("EPD initialization failed!\r\n");
		shell_cmd_finished();
		return CMD_RET_ASYNC;
	} else {
		shell_write("EPD initialized.\r\n");
	}

	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

static int8_t shell_ssd1673_cmd_print(uint8_t argc, char * argv[])
{
	if (argc != 1) {
		return CMD_RET_USAGE;
	}

	shell_writeDec(strlen(argv[0]));

	yafg_print("==== THIS IS A TEST ====", 2, 0, -1);
	yafg_print(argv[0], 3, 0, -1);
	yafg_framebuffer_invert();
	yafg_framebuffer_reverse_bytes();

	ssd1673_set_initial_update();
	ssd1673_write_ram(yafg_get_framebuffer()->data,
			  sizeof(yafg_get_framebuffer()->data));
	ssd1673_do_initial_update();
	/* workaround, FIXME */
	yafg_framebuffer_invert();

	shell_write("-->printed\r\n");
	shell_cmd_finished();

	return CMD_RET_ASYNC;
}

static int8_t shell_ssd1673_cmd_suspend(uint8_t argc, char * argv[])
{
	if (argc != 0)
		return CMD_RET_USAGE;

	shell_write("\n\r--> suspend epd failed\r\n");

	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

#define SHELL_SSD1673_NUMOF_CMD		3

const shell_cmd_table shell_ssd1673_table[SHELL_SSD1673_NUMOF_CMD] = {
	{"init", shell_ssd1673_cmd_init},
	{"print", shell_ssd1673_cmd_print},
	{"suspend", shell_ssd1673_cmd_suspend},
};

const char ssd1673_cmd_help[] = "\r\n"
				"epd init\r\n"
				"epd print\r\n"
				"epd suspend\r\n";

int8_t shell_ssd1673_cmd(uint8_t argc, char *argv[])
{
	if (argc < 2) {
		return CMD_RET_USAGE;
	}

	for (uint8_t i = 0; i < SHELL_SSD1673_NUMOF_CMD; i++) {
		if (!strcmp((char *) argv[1], shell_ssd1673_table[i].name) ) {
			return shell_ssd1673_table[i].cmd(argc - 2,
					(char **) (&argv[2]));
		}
	}

	return CMD_RET_USAGE;
}

const cmd_tbl_t ssd1673_cmd = {
	.name = "epd",
	.maxargs = 3,
	.repeatable = 1,
	.cmd = shell_ssd1673_cmd,
	.usage = (char *) ssd1673_cmd_help,
	.help = "Contains commands for advertising, scanning, "
		"connecting, pairing or disconnecting."
};

int ssd1673_shell_init(void)
{
	shell_register_function((cmd_tbl_t *) &ssd1673_cmd);
	return 0;
}

