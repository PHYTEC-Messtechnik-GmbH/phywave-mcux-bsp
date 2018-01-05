/*
 * Copyright (c) 2017-2018 PHYTEC Messtechnik GmbH
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

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

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
#include "device_info_interface.h"

/* board */
#include "board.h"
#include "clock_config.h"
#include "gpio_pins.h"
#include "pin_mux.h"
#include "i2c_wrapper.h"
#include "ssd1673.h"
#include "tcs37727.h"
#include "hdc1010.h"
#include "ccs811.h"

/* BLE application */
#include "ApplMain.h"

#include "app.h"

#define I2C_MASTER          ((I2C_Type *) I2C1_BASE)
#define I2C_MASTER_CLK_SRC  (I2C1_CLK_SRC)
#define I2C_BAUDRATE        (400000)

#define HDC1010_TIMEOUT_MS	(50/portTICK_PERIOD_MS)

static bool_t adv_state = FALSE;
static deviceId_t peer_device_id = gInvalidDeviceId_c;
static SemaphoreHandle_t xMutex;

static i2c_rtos_handle_t *i2c_handle;
/* HDC1010 */
static hdc1010_t hdc1010_dev;
static int hdc1010_temp;
static int hdc1010_hum;
/* TCS37727 */
static tcs37727_t tcs37727_dev;
static tcs37727_data_t tcs37727_data;
static uint8_t tcs37727_data_8bit[11];
/* CCS811 */
static ccs811_t ccs811_dev;
static uint8_t ccs811_status;
static uint16_t ccs811_eco2;
static uint16_t ccs811_tvoc;

/* initial service configurations */
static disConfig_t dis_service_config = { service_device_info };

/* application specific data */
static tmrTimerID_t app_timer_id;

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
static void char_read_cb(deviceId_t device_id, uint16_t handle,
		uint8_t *value, uint16_t length);
static void char_write_cb(deviceId_t device_id, uint16_t handle,
		uint8_t *value, uint16_t length);

gpioInputPinConfig_t hdc1010_irq_pin_cfg = {
	.gpioPort = HDC1010_INT_FRAMEWORK_GPIO,
	.gpioPin = HDC1010_INT_PIN,
	.pullSelect = pinPull_Up_c,
	.interruptSelect = pinInt_FallingEdge_c
};

static void hdc1010_irq_cb(void)
{
	GpioClearPinIntFlag(&hdc1010_irq_pin_cfg);
	xSemaphoreGive(xMutex);
}

static void hdc1010_setup_interrupt_cb(void)
{
	GpioInputPinInit(&hdc1010_irq_pin_cfg, 1);
	GpioInstallIsr(hdc1010_irq_cb, gGpioIsrPrioLow_c, 0x80, &hdc1010_irq_pin_cfg);
}

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
	BOARD_InitI2C();
	BOARD_InitLPUART();

	if ((i2c_handle = i2c_init_master(I2C_MASTER, I2C_MASTER_CLK_SRC,
					I2C_BAUDRATE)) == NULL) {
		PRINTF("I2C initialization failed\r\n");
		vTaskSuspend(NULL);
	}

	hdc1010_setup_interrupt_cb();
	xMutex = xSemaphoreCreateBinary();
	if (hdc1010_init(&hdc1010_dev, i2c_handle, HDC1010_I2C_ADDRESS)) {
		PRINTF("HDC1010 initialization failed\r\n");
		vTaskSuspend(NULL);
	}

	if (tcs37727_init(&tcs37727_dev, i2c_handle, TCS37727_I2C_ADDRESS,
				TCS37727_ATIME_DEFAULT)) {
		PRINTF("TCS37727 initialization failed\r\n");
		vTaskSuspend(NULL);
	}

	if (tcs37727_set_rgbc_active(&tcs37727_dev)) {
		PRINTF("TCS37727 activation failed\r\n");
		vTaskSuspend(NULL);
	}

	int err = ccs811_init(&ccs811_dev, i2c_handle, CCS811_I2C_ADDRESS);
	if (err != 0) {
		PRINTF("CCS811 initialization failed with error %d\r\n", err);
	}
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
		App_StartAdvertising(gap_advertising_cb, gap_connection_cb);
		break;
	default:
		break;
	}
}

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
		value_epaper_update,
		value_sensors_ctrl,
		value_temp,
		value_env,
		value_light
	};
	uint8_t handle_count = sizeof(handle_array) / sizeof(uint16_t);
	GattServer_RegisterHandlesForWriteNotifications(handle_count,
			(uint16_t *) &handle_array[0]);
	GattServer_RegisterHandlesForReadNotifications(handle_count,
			(uint16_t *) &handle_array[0]);
	App_RegisterGattServerCallback(gatt_server_cb);

	/* start services */
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

			if (gBleSuccess_c == Gap_CheckIfBonded(device_id,
						&isBonded) &&
					true == isBonded) {
			}
#endif

			/* advertising stops when connected */
			adv_state = FALSE;
			TMR_StopTimer(app_timer_id);

			/* subscribe client */
			peer_device_id = device_id;

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

#if (cPWR_UsePowerDownMode)
			/* UI */
			Led1Off();

			/* go to sleep */
			PWR_ChangeDeepSleepMode(3);
			PWR_SetDeepSleepTimeInMs(cPWR_DeepSleepDurationMs);
			PWR_AllowDeviceToSleep();
#endif
			advertise();
		}
		break;
	case gConnEvtEncryptionChanged_c:
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
		char_read_cb(device_id, hnd, val, len);
		GattServer_SendAttributeReadStatus(device_id, hnd,
				gAttErrCodeNoError_c);
		break;
	case gEvtAttributeWrittenWithoutResponse_c:
		char_write_cb(device_id, hnd, val, len);
		break;
	case gEvtAttributeWritten_c:
		char_write_cb(device_id, hnd, val, len);
		GattServer_SendAttributeWrittenStatus(device_id, hnd,
				gAttErrCodeNoError_c);
		break;
	case gEvtCharacteristicCccdWritten_c:
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

static void char_read_cb(deviceId_t device_id, uint16_t handle,
		uint8_t *value, uint16_t length)
{
	switch (handle) {
	case value_sensors_ctrl:
		break;
	case value_temp:
		if (hdc1010_startmeasure(&hdc1010_dev)) {
			PRINTF("HDC1010 starting measurement failed\r\n");
			return;
		}

		if (xSemaphoreTake(xMutex, (TickType_t) HDC1010_TIMEOUT_MS)) {
			uint16_t raw_temp;
			uint16_t raw_hum;
			hdc1010_read(&hdc1010_dev, &raw_temp, &raw_hum);
			hdc1010_convert(raw_temp, raw_hum, &hdc1010_temp, &hdc1010_hum);
			uint8_t values[4] = {
				hdc1010_temp,
				hdc1010_temp >> 8,
				hdc1010_hum,
				hdc1010_hum >> 8,
			};
			GattDb_WriteAttribute(value_temp, sizeof(values), values);
		} else {
			PRINTF("HDC1010 timeout while reading\r\n");
		}
		break;
	case value_light: {
		uint32_t red, green, blue, lux, ct;
		tcs37727_read(&tcs37727_dev, &tcs37727_data);
		red = tcs37727_data.red;
		green = tcs37727_data.green;
		blue = tcs37727_data.blue;
		lux = tcs37727_data.lux;
		ct = tcs37727_data.ct;

		// rgb
		int div = 0;
		if (red < green) {
			if (blue < green) {
				div = green / 256;
			} else {
				div = blue / 256;
			}
		} else if (red < blue) {
			if (green < blue) {
				div = blue / 256;
			} else {
				div = green / 256;
			}
		} else {
			div = red / 256;
		}
		div++;
		red /= div;
		green /= div;
		blue /= div;
		tcs37727_data_8bit[0] = red;
		tcs37727_data_8bit[1] = green;
		tcs37727_data_8bit[2] = blue;

		// lux
		tcs37727_data_8bit[3] = lux;
		tcs37727_data_8bit[4] = lux >> 8;
		tcs37727_data_8bit[5] = lux >> 16;
		tcs37727_data_8bit[6] = lux >> 24;

		// ct
		tcs37727_data_8bit[7] = ct;
		tcs37727_data_8bit[8] = ct >> 8;
		tcs37727_data_8bit[9] = ct >> 16;
		tcs37727_data_8bit[10] = ct >> 24;

		GattDb_WriteAttribute(value_light, sizeof(tcs37727_data_8bit),
				tcs37727_data_8bit);
		}
		break;
	case value_env:
		ccs811_resume(&ccs811_dev);
		vTaskDelay(1000);
		if (ccs811_read(&ccs811_dev, &ccs811_eco2, &ccs811_tvoc,
					&ccs811_status)) {
			PRINTF("CCS811 read failed or not ready\r\n");
			return;
		} else {
			uint8_t ccs811_meas[5] = {
				ccs811_eco2,
				ccs811_eco2 >> 8,
				ccs811_tvoc,
				ccs811_tvoc >> 8,
				ccs811_status
			};
			GattDb_WriteAttribute(value_env, sizeof(ccs811_meas), ccs811_meas);
		}
		break;
	default:
		break;
	}
}

static void char_write_cb(deviceId_t device_id, uint16_t handle,
		uint8_t *value, uint16_t length)
{
	GattDb_WriteAttribute(handle, length, value);

	switch (handle) {
	case value_sensors_ctrl:
		if (*value == 0x10) {
			ccs811_resume(&ccs811_dev);
		} else if (*value == 0x11) {
			ccs811_suspend(&ccs811_dev);
		}
		break;
	case value_epaper_init:
		ssd1673_init();
		ssd1673_set_initial_update();
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
