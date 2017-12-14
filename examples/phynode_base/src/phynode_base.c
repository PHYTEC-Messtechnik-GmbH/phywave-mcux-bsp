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
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "fsl_i2c_freertos.h"

#include "shell.h"

#include "Keyboard.h"
#include "PWR_Interface.h"
#include "PWR_Configuration.h"

#include "board.h"
#include "pin_mux.h"
#include "gpio_pins.h"
#include "clock_config.h"

#include "ssd1673_shell.h"
#include "hdc1010_shell.h"
#include "tcs37727_shell.h"
#include "ccs811_shell.h"
#include "i2c_wrapper.h"
#include "adc_soc.h"

#define I2C_MASTER_BASE         (I2C1_BASE)
#define I2C_MASTER_CLK_SRC      (I2C1_CLK_SRC)
#define I2C_MASTER              ((I2C_Type*)I2C_MASTER_BASE)
#define I2C_BAUDRATE            (100000)

#define MASTER_TASK_PRIORITY    (configMAX_PRIORITIES - 2)

typedef struct {
	char* name;
	int8_t (*cmd)(uint8_t argc, char * argv[]);
} shell_cmd_table;

static int8_t shell_pwr_timer_sleep(uint8_t argc, char * argv[])
{
	PWRLib_WakeupReason_t wakeupReason;

	if (argc != 0)
		return CMD_RET_USAGE;

	PWR_ChangeDeepSleepMode(3);
	PWR_SetDeepSleepTimeInMs(cPWR_DeepSleepDurationMs);
	PWR_AllowDeviceToSleep();

	if(PWR_CheckIfDeviceCanGoToSleep()) {
		shell_write("--> enter DSM 3\n\r");
		phynode_v_periph_disable();
		wakeupReason = PWR_EnterLowPower();
		phynode_v_periph_enable();
		shell_write("--> I am awake: ");
		shell_writeHexLe((uint8_t*)&wakeupReason, sizeof(wakeupReason));
		shell_write("\n\r");
	} else {
		shell_write("--> can not enter low power\n\r");
	}

	shell_cmd_finished();
	return CMD_RET_ASYNC;
}

#define SHELL_BOARD_NUMOF_CMD			1

const shell_cmd_table shell_board_table[SHELL_BOARD_NUMOF_CMD] =
{
	{"sleep", shell_pwr_timer_sleep},
};

const char board_cmd_help[] = "\r\n"
			 "sleep\r\n";

int8_t shell_board_cmd(uint8_t argc, char * argv[])
{
	uint8_t i;

	if (argc < 2) {
		return CMD_RET_USAGE;
	}

	for (i = 0; i < SHELL_BOARD_NUMOF_CMD; i++) {
		if (!strcmp((char*)argv[1], shell_board_table[i].name) ) {
			return shell_board_table[i].cmd(argc - 2, (char**)(&argv[2]));
		}
	}

	return CMD_RET_USAGE;
}

const cmd_tbl_t board_cmd =
{
	.name		= "board",
	.maxargs	= 3,
	.repeatable	= 1,
	.cmd		= shell_board_cmd,
	.usage		= (char*)board_cmd_help,
	.help		= "Commands for the phyNODE demo functions"
};

static void App_KeyboardCallBack(uint8_t events)
{
	(void)events;
}

static void main_task(void *pvParameters)
{
	shell_cmd_finished();
	vTaskSuspend(NULL);
}

int main(void)
{
	i2c_rtos_handle_t *i2c_handle;

	BOARD_InitLPUART();
	hardware_init();
	BOARD_InitGPIO();
	BOARD_InitI2C();
	BOARD_InitSPI();
	BOARD_InitAdc();
        MEM_Init();
        TMR_Init();
	KBD_Init(App_KeyboardCallBack);
	PWR_DisallowDeviceToSleep();
	PWR_Init();

	i2c_handle = i2c_init_master(I2C_MASTER, I2C_MASTER_CLK_SRC,
				     I2C_BAUDRATE);
	if (i2c_handle == NULL) {
		return -1;
	}

	shell_init("phyNODE:>");
	shell_register_function((cmd_tbl_t*)&board_cmd);
	hdc1010_shell_init(i2c_handle);
	tcs37727_shell_init(i2c_handle);
	ccs811_shell_init(i2c_handle);
	ssd1673_shell_init();

	if (xTaskCreate(main_task, "phyNODE",
			configMINIMAL_STACK_SIZE + 60,
			NULL, MASTER_TASK_PRIORITY, NULL) != pdPASS) {
		return -1;
	}

	vTaskStartScheduler();
	return 0;
}
