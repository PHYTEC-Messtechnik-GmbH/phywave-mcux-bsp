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

PRIMARY_SERVICE(service_gatt, gBleSig_GenericAttributeProfile_d)
	CHARACTERISTIC(char_service_changed, gBleSig_GattServiceChanged_d, (gGattCharPropRead_c | gGattCharPropNotify_c))
		VALUE(value_service_changed, gBleSig_GattServiceChanged_d, (gPermissionNone_c), 4, 0x00, 0x00, 0x00, 0x00)
		CCCD(cccd_service_changed)

PRIMARY_SERVICE(service_gap, gBleSig_GenericAccessProfile_d)
	CHARACTERISTIC(char_device_name, gBleSig_GapDeviceName_d, (gGattCharPropRead_c))
		VALUE(value_device_name, gBleSig_GapDeviceName_d, (gPermissionFlagReadable_c), 12, "FSL_BLE_TEMP")

PRIMARY_SERVICE_UUID128(service_temperature, uuid_service_temperature)
	CHARACTERISTIC(char_temperature, gBleSig_Temperature_d, (gGattCharPropNotify_c | gGattCharPropRead_c))
		VALUE(value_temperature, gBleSig_Temperature_d, (gPermissionFlagReadable_c), 2, 0x00, 0xB4)
		DESCRIPTOR(desc_temperature, gBleSig_CharPresFormatDescriptor_d, (gPermissionFlagReadable_c), 7, 0x0E, 0xFE, 0x2F, 0x27, 0x00, 0x00, 0x00)
		CCCD(cccd_temperature)

PRIMARY_SERVICE(service_battery, gBleSig_BatteryService_d)
	CHARACTERISTIC(char_battery_level, gBleSig_BatteryLevel_d, (gGattCharPropNotify_c | gGattCharPropRead_c))
		VALUE(value_battery_level, gBleSig_BatteryLevel_d, (gPermissionFlagReadable_c), 1, 0x5A)
		DESCRIPTOR(desc_bat_level, gBleSig_CharPresFormatDescriptor_d, (gPermissionFlagReadable_c), 7, 0x04, 0x00, 0xAD, 0x27, 0x01, 0x01, 0x00)
		CCCD(cccd_battery_level)

PRIMARY_SERVICE_UUID128(service_epaper, uuid_service_epaper)
	CHARACTERISTIC_UUID128(char_epaper_init, uuid_char_epaper_init, (gGattCharPropRead_c | gGattCharPropWrite_c))
		VALUE_UUID128(value_epaper_init, uuid_char_epaper_init, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 1, 0x00)
	CHARACTERISTIC_UUID128(char_epaper_data, uuid_char_epaper_data, (gGattCharPropRead_c | gGattCharPropWrite_c | gGattCharPropWriteWithoutRsp_c))
		VALUE_UUID128(value_epaper_data, uuid_char_epaper_data, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00)
	CHARACTERISTIC_UUID128(char_epaper_update, uuid_char_epaper_update, (gGattCharPropRead_c | gGattCharPropWrite_c))
		VALUE_UUID128(value_epaper_update, uuid_char_epaper_update, (gPermissionFlagReadable_c | gPermissionFlagWritable_c), 1, 0x00)

PRIMARY_SERVICE(service_device_info, gBleSig_DeviceInformationService_d)
	CHARACTERISTIC(char_manuf_name, gBleSig_ManufacturerNameString_d, (gGattCharPropRead_c))
		VALUE(value_manuf_name, gBleSig_ManufacturerNameString_d, (gPermissionFlagReadable_c), sizeof(MANUFACTURER_NAME), MANUFACTURER_NAME)
	CHARACTERISTIC(char_model_no, gBleSig_ModelNumberString_d, (gGattCharPropRead_c))
		VALUE(value_model_no, gBleSig_ModelNumberString_d, (gPermissionFlagReadable_c), 16, "Temp Sensor Demo")
	CHARACTERISTIC(char_serial_no, gBleSig_SerialNumberString_d, (gGattCharPropRead_c))
		VALUE(value_serial_no, gBleSig_SerialNumberString_d, (gPermissionFlagReadable_c), 7, "BLESN01")
	CHARACTERISTIC(char_hw_rev, gBleSig_HardwareRevisionString_d, (gGattCharPropRead_c))
		VALUE(value_hw_rev, gBleSig_HardwareRevisionString_d, (gPermissionFlagReadable_c), sizeof(BOARD_NAME), BOARD_NAME)
	CHARACTERISTIC(char_fw_rev, gBleSig_FirmwareRevisionString_d, (gGattCharPropRead_c))
		VALUE(value_fw_rev, gBleSig_FirmwareRevisionString_d, (gPermissionFlagReadable_c), 5, "1.1.1")
	CHARACTERISTIC(char_sw_rev, gBleSig_SoftwareRevisionString_d, (gGattCharPropRead_c))
		VALUE(value_sw_rev, gBleSig_SoftwareRevisionString_d, (gPermissionFlagReadable_c), 5, "1.1.4")
