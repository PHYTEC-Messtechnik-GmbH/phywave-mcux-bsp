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

#include "fsl_debug_console.h"

/* framework */
#include "FunctionLib.h"
#include "Keyboard.h"
#include "LED.h"
#include "MemManager.h"
#include "Panic.h"
#include "RNG_Interface.h"
#include "TimersManager.h"

#if cPWR_UsePowerDownMode
#include "PWR_Configuration.h"
#include "PWR_Interface.h"
#endif

/* BLE */
#include "ble_conn_manager.h"
#include "gap_interface.h"
#include "gatt_client_interface.h"
#include "gatt_db_app_interface.h"
#include "gatt_db_handles.h"
#include "gatt_server_interface.h"

/* profile, services */
#include "battery_interface.h"
#include "device_info_interface.h"
#include "temperature_interface.h"

/* board */
#include "board.h"
#include "clock_config.h"
#include "gpio_pins.h"
#include "pin_mux.h"
#include "ssd1673.h"

/* BLE application */
#include "ApplMain.h"

#include "app.h"

/* battery level report interval in seconds */
#define mBatteryLevelReportInterval_c  (10)

typedef struct {
	bool_t advOn;
} advState_t;
static bool_t adv_state = FALSE;

static deviceId_t peer_device_id = gInvalidDeviceId_c;

/* initial service configurations */
static basConfig_t bas_service_config = { service_battery, 0 };
static disConfig_t dis_service_config = { service_device_info };
static tmsConfig_t tms_service_config = { service_temperature, 0 };

/* application specific data */
static tmrTimerID_t app_timer_id;

static bool_t send_data_after_enc_start = FALSE;

/* GATT and ATT callbacks */
static void gap_advertising_cb(
		gapAdvertisingEvent_t *event);
static void gap_connection_cb(deviceId_t device_id,
		gapConnectionEvent_t *event);
static void gatt_server_cb(deviceId_t device_id,
		gattServerEvent_t *event);
static void config(void);

/* timer callbacks */
#if cPWR_UsePowerDownMode
static void advertising_timer_cb(void *);
static void disconnect_timer_cb(void *);
#endif

static void advertise(void);
static void temperature_send(void);

static void epaper_init(void);
static void epaper_write_cb(deviceId_t device_id, uint16_t handle,
		uint8_t *value, uint16_t length);

/* global ------------------------------------------------------------------- */

/**
 * Initializes application specific functionality before the BLE stack init.
 */
void BleApp_Init(void)
{
	hardware_init();
#if cPWR_UsePowerDownMode
	PWR_DisallowDeviceToSleep();
	PWR_Init();
#endif
	BOARD_InitDebugConsole();
	BOARD_InitGPIO();
	BOARD_InitPins();
	BOARD_InitSPI();
	BOARD_InitLPUART();
}

/**
 * Starts the BLE application.
 */
void BleApp_Start(void)
{
	Led1On();

	if (peer_device_id == gInvalidDeviceId_c) {
		/* device is not connected and not advertising */
		if (!adv_state) {
			advertise();
		}

#if (cPWR_UsePowerDownMode)
		PWR_ChangeDeepSleepMode(1);
		PWR_AllowDeviceToSleep();
#endif
	} else {
		temperature_send();
	}
}

/**
 * Handles keyboard events.
 */
void BleApp_HandleKeys(key_event_t events)
{
	BleApp_Start();
}

/**
 * Handles BLE generic callback.
 */
void BleApp_GenericCallback(gapGenericEvent_t *event)
{
	/* call BLE connection manager */
	BleConnManager_GenericEvent(event);

	switch (event->eventType) {
	case gInitializationComplete_c:
		config();
		break;
	case gAdvertisingParametersSetupComplete_c:
		App_StartAdvertising(gap_advertising_cb,
				gap_connection_cb);
		break;
	default:
		break;
	}
}

/* local -------------------------------------------------------------------- */

/**
 * Configures BLE Stack after initialization. Usually used for configuring
 * advertising, scanning, white list, services, et al.
 */
static void config(void)
{
	adv_state = FALSE;

	/* configure as GAP peripheral */
	BleConnManager_GapPeripheralConfig();

	/* register callbacks */
	uint16_t handle_array[] = {
		value_epaper_init,
		value_epaper_data,
		value_epaper_update
	};
	uint8_t handle_count = sizeof(handle_array) / sizeof(uint16_t);
	GattServer_RegisterHandlesForWriteNotifications(handle_count,
			(uint16_t *) &handle_array[0]);
	GattServer_RegisterHandlesForReadNotifications(handle_count,
			(uint16_t *) &handle_array[0]);
	App_RegisterGattServerCallback(gatt_server_cb);

	/* start services */
	Tms_Start(&tms_service_config);
	Bas_Start(&bas_service_config);
	Dis_Start(&dis_service_config);

	/* allocate application timer */
	app_timer_id = TMR_AllocateTimer();

#if (cPWR_UsePowerDownMode)
	PWR_ChangeDeepSleepMode(3);
	PWR_AllowDeviceToSleep();
#endif
}

/**
 * Configures GAP Advertise parameters. Advertise will start after the
 * parameters are set.
 */
static void advertise(void)
{
	/* Set advertising parameters*/
	Gap_SetAdvertisingParameters(&gAdvParams);
}

/**
 * Handles BLE Advertising callback from host stack.
 */
static void gap_advertising_cb(gapAdvertisingEvent_t *event)
{
	switch (event->eventType) {
	case gAdvertisingStateChanged_c:
		{
			adv_state = !adv_state;

#if (cPWR_UsePowerDownMode)
			if (!adv_state) {
				Led1Off();
				PWR_ChangeDeepSleepMode(3);
				PWR_SetDeepSleepTimeInMs(cPWR_DeepSleepDurationMs);
				PWR_AllowDeviceToSleep();
			} else {
				/* start advertising timer */
				TMR_StartLowPowerTimer(app_timer_id,
						gTmrLowPowerSecondTimer_c,
						TmrSeconds(gAdvTime_c),
						advertising_timer_cb, NULL);

				Led1On();
			}
#else
			LED_StopFlashingAllLeds();
			Led1Flashing();

			if (!adv_state) {
				Led2Flashing();
				Led3Flashing();
				Led4Flashing();
			}
#endif
		}
		break;

	case gAdvertisingCommandFailed_c:
		{
			Led2On();
			panic(0, 0, 0, 0);
		}
		break;

	default:
		break;
	}
}

/**
 * Handles BLE Connection callback from host stack.
 */
static void gap_connection_cb(deviceId_t device_id,
		gapConnectionEvent_t* event)
{
	/* connection manager to handle host stack interactions */
	BleConnManager_GapPeripheralEvent(device_id, event);

	switch (event->eventType) {
	case gConnEvtConnected_c:
		{
#if gAppUseBonding_d
			bool_t isBonded = FALSE;

			send_data_after_enc_start = FALSE;
			if (gBleSuccess_c == Gap_CheckIfBonded(device_id,
						&isBonded) &&
					true == isBonded) {
				/* send temperature data after encryption has
				 * started
				 */
				send_data_after_enc_start = TRUE;
			}
#endif

			/* advertising stops when connected */
			adv_state = FALSE;
			TMR_StopTimer(app_timer_id);

			/* subscribe client */
			peer_device_id = device_id;
			Bas_Subscribe(device_id);
			Tms_Subscribe(device_id);

			/* UI */
			Led1On();

#if (cPWR_UsePowerDownMode)
			PWR_ChangeDeepSleepMode(1);
			PWR_SetDeepSleepTimeInMs(cPWR_DeepSleepDurationMs);
			PWR_AllowDeviceToSleep();
#else
			LED_StopFlashingAllLeds();

#endif
		}
		break;

	case gConnEvtDisconnected_c:
		{
			/* unsubscribe client */
			peer_device_id = gInvalidDeviceId_c;
			Bas_Unsubscribe();
			Tms_Unsubscribe();

#if (cPWR_UsePowerDownMode)
			/* UI */
			Led1Off();

			/* go to sleep */
			PWR_ChangeDeepSleepMode(3);
			PWR_SetDeepSleepTimeInMs(cPWR_DeepSleepDurationMs);
			PWR_AllowDeviceToSleep();
#endif
		}
		break;
	case gConnEvtEncryptionChanged_c:
		{
			if (send_data_after_enc_start) {
				temperature_send();
			}
		}
		break;
	default:
		break;
	}
}

/**
 * Handles GATT server callbacks from host stack.
 */
void gatt_server_cb(deviceId_t device_id, gattServerEvent_t *event)
{
	uint16_t hnd = event->eventData.attributeWrittenEvent.handle;
	uint16_t len = event->eventData.attributeWrittenEvent.cValueLength;
	uint8_t *val = event->eventData.attributeWrittenEvent.aValue;

	switch (event->eventType) {
	case gEvtAttributeRead_c:
		GattServer_SendAttributeReadStatus(device_id, value_epaper_init,
				gAttErrCodeNoError_c);
		break;
	case gEvtAttributeWrittenWithoutResponse_c:
		epaper_write_cb(device_id, hnd, val, len);
		break;
	case gEvtAttributeWritten_c:
		epaper_write_cb(device_id, hnd, val, len);
		GattServer_SendAttributeWrittenStatus(device_id, hnd,
				gAttErrCodeNoError_c);
		break;
	case gEvtCharacteristicCccdWritten_c:
		temperature_send();
		break;
	case gEvtError_c:
		break;
	default:
		break;
	}
}

#if cPWR_UsePowerDownMode
/**
 * Handles advertising timer callback.
 */
static void advertising_timer_cb(void *pParam)
{
	/* stop advertising */
	if (adv_state) {
		Gap_StopAdvertising();
	}
}
#endif

#if cPWR_UsePowerDownMode
/**
 * Handles disconnect timer callback.
 */
static void disconnect_timer_cb(void *pParam)
{
	if (peer_device_id != gInvalidDeviceId_c) {
		Gap_Disconnect(peer_device_id);
	}
}
#endif

/**
 * Sends temperature value over-the-air.
 */
static void temperature_send(void)
{
	TMR_StopTimer(app_timer_id);
	/* update with initial temperature */
	Tms_RecordTemperatureMeasurement(service_temperature, 42);

#if (cPWR_UsePowerDownMode)
	/* start sleep after data timer */
	TMR_StartLowPowerTimer(app_timer_id, gTmrLowPowerSecondTimer_c,
			TmrSeconds(gGoToSleepAfterDataTime_c),
			disconnect_timer_cb, NULL);
	/* go to sleep */
	PWR_SetDeepSleepTimeInMs(TmrSeconds(gGoToSleepAfterDataTime_c));
#endif
}

static void epaper_init(void)
{
	/* FIXME: check the return value of ssd1673_init()! */
	ssd1673_init();
	ssd1673_set_initial_update();
}

static void epaper_write_cb(deviceId_t device_id, uint16_t handle,
		uint8_t *value, uint16_t length)
{
	GattDb_WriteAttribute(handle, length, value);

	switch (handle) {
	case value_epaper_init:
		epaper_init();
		break;
	case value_epaper_data:
		ssd1673_write_ram(value, length);
		break;
	case value_epaper_update:
		ssd1673_do_initial_update();
		break;
	default:
		break;
	}
}
