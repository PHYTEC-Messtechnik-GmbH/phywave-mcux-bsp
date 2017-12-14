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

#include "EmbeddedTypes.h"
#include "fsl_device_registers.h"
#include "fsl_common.h"
#include "fsl_clock.h"
#include "fsl_smc.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"

#include "GPIO_Adapter.h"
#include "RNG_Interface.h"
#include "Flash_Adapter.h"

#include "clock_config.h"
#include "board.h"
#include "pin_mux.h"
#include "periph.h"
#include "gpio_pins.h"

#if cPWR_UsePowerDownMode
#include "PWR_Interface.h"
#endif

#if gDCDC_Enabled_d
#include "DCDC.h"
#endif

#include "FunctionLib.h"
#include "Keyboard.h"

static const dcdcConfig_t mDcdcDefaultConfig =
{
#if APP_DCDC_MODE == gDCDC_Mode_Buck_c
	.vbatMin = 1800,
	.vbatMax = 4200,
#elif APP_DCDC_MODE == gDCDC_Mode_Boost_c
	.vbatMin = 900,
	.vbatMax = 1800,
#endif
	.dcdcMode = APP_DCDC_MODE,
	.vBatMonitorIntervalMs = APP_DCDC_VBAT_MONITOR_INTERVAL,
	.pfDCDCAppCallback = NULL,
	.dcdcMcuVOutputTargetVal = gDCDC_McuV_OutputTargetVal_1_500_c,
	.dcdc1P8OutputTargetVal = gDCDC_1P8OutputTargetVal_3_000_c,
};

static void DCDC_SetupVbatAdcDivider(uint8_t divider)
{
	if (divider == 4u) {
		divider = 3u;
	}
	DCDC->REG0 = (DCDC->REG0 & ~DCDC_REG0_DCDC_VBAT_DIV_CTRL_MASK ) |
		      DCDC_REG0_DCDC_VBAT_DIV_CTRL(divider);
}

void BOARD_InitDebugConsole(void)
{
	uint32_t uartClkSrcFreq;

	/* SIM_SOPT2[27:26]:
	 *  00: Clock Disabled
	 *  01: MCGFLLCLK
	 *  10: OSCERCLK
	 *  11: MCGIRCCLK
	 */
	CLOCK_SetLpuartClock(2);

	uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;

	DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR,
			BOARD_DEBUG_UART_BAUDRATE,
			BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
}

void BOARD_EnterLowPowerCb()
{
#if gKeyBoardSupported_d
	KBD_PrepareEnterLowPower();
#endif
	BOARD_SetPinsLowPower(TRUE);
#if gDCDC_Enabled_d
	DCDC_SetupVbatAdcDivider(0);
	DCDC_PrepareForPulsedMode();
#endif
}

void BOARD_ExitLowPowerCb()
{
#if gDCDC_Enabled_d
	DCDC_PrepareForContinuousMode();
	DCDC_SetupVbatAdcDivider(DCDC_VBAT_ADC_DIVIDER);
#endif
#if gKeyBoardSupported_d
	KBD_PrepareExitLowPower();
#endif
	BOARD_SetPinsLowPower(FALSE);

}

void phynode_v_periph_enable(void)
{
	GPIO_WritePinOutput(V_PERIPH_GPIO_DEV, V_PERIPH_PIN, 0);
}

void phynode_v_periph_disable(void)
{
	GPIO_WritePinOutput(V_PERIPH_GPIO_DEV, V_PERIPH_PIN, 1);
}

void hardware_init(void)
{
	static uint8_t initialized = 0;
	uint32_t temp;

	if ( !initialized ) {
		if (SMC->PMPROT == 0x00) {
			SMC->PMPROT = SYSTEM_SMC_PMPROT_VALUE;
		}

		if ((PMC->REGSC & PMC_REGSC_ACKISO_MASK) != 0x00U) {
			/* Release hold with ACKISO:  Only has an effect if recovering from VLLSx.*/
			PMC->REGSC |= PMC_REGSC_ACKISO_MASK;
			/*clear power management registers after recovery from vlls*/
			SMC->STOPCTRL &= ~SMC_STOPCTRL_LLSM_MASK;
			SMC->PMCTRL   &= ~(SMC_PMCTRL_STOPM_MASK | SMC_PMCTRL_RUNM_MASK);
		}

		SIM->SCGC6 |= (SIM_SCGC6_DMAMUX_MASK); /* Enable clock to DMA_MUX (SIM module) */
		SIM->SCGC7 |= (SIM_SCGC7_DMA_MASK);

		/* Turn on clocks for the XCVR */
		/* Enable RF OSC in RSIM and wait for ready */
		temp = RSIM->CONTROL;
		temp &= ~RSIM_CONTROL_RF_OSC_EN_MASK;
		RSIM->CONTROL = temp | RSIM_CONTROL_RF_OSC_EN(1);
		/* Prevent XTAL_OUT_EN from generating XTAL_OUT request */
		RSIM->RF_OSC_CTRL |= RSIM_RF_OSC_CTRL_RADIO_EXT_OSC_OVRD_EN_MASK;
		/* wait for RF_OSC_READY */
		while ((RSIM->CONTROL & RSIM_CONTROL_RF_OSC_READY_MASK) == 0) {
		}

		BOARD_InitPins();
		BOARD_BootClockRUN();
		DCDC_Init(&mDcdcDefaultConfig);
		DCDC_SetupVbatAdcDivider(DCDC_VBAT_ADC_DIVIDER);

		/* Install callbacks to handle enter and exit low power */
#if cPWR_UsePowerDownMode
		PWR_RegisterLowPowerEnterCallback((pfPWRCallBack_t)BOARD_EnterLowPowerCb);
		PWR_RegisterLowPowerExitCallback((pfPWRCallBack_t)BOARD_ExitLowPowerCb);
#endif

		NV_ReadHWParameters(&gHardwareParameters);
		if (0xFFFFFFFF == gHardwareParameters.xtalTrim) {
			gHardwareParameters.xtalTrim = BOARD_XTAL_TRIM_DEFAULT;
		}
		initialized = 1;
	}
}

/* get 4 words of information that uniquely identifies the MCU */
void BOARD_GetMCUUid(uint8_t* aOutUid16B, uint8_t* pOutLen)
{
	uint32_t uid[4] = { 0 };

	uid[0] = SIM->SDID;
	uid[1] = SIM->UIDMH;
	uid[2] = SIM->UIDML;
	uid[3] = SIM->UIDL;

	FLib_MemCpy(aOutUid16B, (uint8_t*)uid, sizeof(uid));
	*pOutLen = sizeof(uid);

	return;
}

uint32_t BOARD_GetLpuartClock(uint32_t instance)
{
	uint32_t clock;
	uint32_t clockSource = (SIM->SOPT2 & SIM_SOPT2_LPUART0SRC_MASK) >> SIM_SOPT2_LPUART0SRC_SHIFT;

	instance = instance; /* Remove compiler warnings */

	switch (clockSource) {
	case 1: /* MCGFLLCLK */
		clock = CLOCK_GetFllFreq();
		break;
	case 2: /* OSCERCLK */
		clock = CLOCK_GetOsc0ErClkFreq();
		break;
	case 3: /* MCGIRCLK */
		clock = CLOCK_GetInternalRefClkFreq();
		break;
	default: /* Clock disabled */
		clock = 0;
		break;
	}

	return clock;
}

uint32_t BOARD_GetTpmClock(uint32_t instance)
{
	uint32_t clock;
	uint32_t clockSource = (SIM->SOPT2 & SIM_SOPT2_TPMSRC_MASK) >> SIM_SOPT2_TPMSRC_SHIFT;

	instance = instance; /* Remove compiler warnings */

	switch (clockSource) {
	case 1: /* MCGFLLCLK */
		clock = CLOCK_GetFllFreq();
		break;
	case 2: /* OSCERCLK */
		clock = CLOCK_GetOsc0ErClkFreq();
		break;
	case 3: /* MCGIRCLK */
		clock = CLOCK_GetInternalRefClkFreq();
		break;
	default: /* Clock disabled */
		clock = 0;
		break;
	}

	return clock;
}

uint32_t BOARD_GetSpiClock(uint32_t instance)
{
	instance = instance; /* Remove compiler warnings */
	return CLOCK_GetFreq(kCLOCK_BusClk);
}

uint32_t BOARD_GetI2cClock(uint32_t instance)
{
	uint32_t clock;

	switch (instance) {
	case 1:
		clock = CLOCK_GetFreq(kCLOCK_CoreSysClk);
		break;

	default:
		clock = CLOCK_GetFreq(kCLOCK_BusClk);
		break;
	}

	return clock;
}

void BOARD_BLPEtoBLPI(void)
{
}

void BOARD_BLPItoBLPE(void)
{
	while ((RSIM->CONTROL & RSIM_CONTROL_RF_OSC_READY_MASK) == 0 ) {
	}
}
