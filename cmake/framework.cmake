#
# Copyright (c) 2017 PHYTEC Messtechnik GmbH
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the PHYTEC Messtechnik GmbH nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#

set(MCUX_FRAMEWORK_DIR ${MCUX_SDK_DIR}/middleware/wireless/framework_${MCUX_FRAMEWORK_VERSION})
set(MCUX_BLUETOOTH_DIR ${MCUX_SDK_DIR}/middleware/wireless/bluetooth_${MCUX_BLUETOOTH_VERSION})
set(MCUX_FREERTOS_DIR ${MCUX_SDK_DIR}/rtos/freertos_${MCUX_FREERTOS_VERSION})

set(MCUX_MIDDLEWARE_LIBS
"${MCUX_BLUETOOTH_DIR}/controller/lib/lib_ble_kw41z_controller.a"
"${MCUX_BLUETOOTH_DIR}/host/lib/lib_ble_4-2_host_cm0p.a"
"${MCUX_FRAMEWORK_DIR}/SecLib/lib_crypto_m0.a"
)

target_include_directories(${TARGET} PUBLIC
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/SerialManager/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/FunctionLib>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/TimersManager/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/OSAbstraction/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/LowPower/Interface/MKW41Z/>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Common>
)

if(BLUETOOTH_ENABLED)
  target_sources(${TARGET} PUBLIC
  "${MCUX_BLUETOOTH_DIR}/application/common/ApplMain.c"
  "${MCUX_BLUETOOTH_DIR}/application/common/ble_conn_manager.c"
  "${MCUX_BLUETOOTH_DIR}/application/common/ble_controller_task.c"
  "${MCUX_BLUETOOTH_DIR}/application/common/ble_host_tasks.c"
  "${MCUX_BLUETOOTH_DIR}/application/common/ble_init.c"
  "${MCUX_BLUETOOTH_DIR}/host/config/ble_globals.c"
  #TODO HCI:
  "${MCUX_BLUETOOTH_DIR}/hci_transport/source/hcit_serial_interface.c"
  #TODO GATT:
  )
endif(BLUETOOTH_ENABLED)

if(BLUETOOTH_ENABLED AND BLUETOOTH_GATT_ENABLED)
  target_sources(${TARGET} PUBLIC
  "${MCUX_BLUETOOTH_DIR}/application/common/gatt_db/gatt_database.c"
  "${MCUX_BLUETOOTH_DIR}/application/common/gatt_db/gatt_database_dynamic.c"
  "${MCUX_BLUETOOTH_DIR}/profiles/battery/battery_service.c"
  "${MCUX_BLUETOOTH_DIR}/profiles/device_info/device_info_service.c"
  "${MCUX_BLUETOOTH_DIR}/profiles/temperature/temperature_service.c"
  )
endif(BLUETOOTH_ENABLED AND BLUETOOTH_GATT_ENABLED)

target_include_directories(${TARGET} PUBLIC
$<BUILD_INTERFACE:${MCUX_BLUETOOTH_DIR}/application/common>
$<BUILD_INTERFACE:${MCUX_BLUETOOTH_DIR}/application/common/gatt_db>
$<BUILD_INTERFACE:${MCUX_BLUETOOTH_DIR}/application/common/gatt_db/macros>
$<BUILD_INTERFACE:${MCUX_BLUETOOTH_DIR}/controller/interface>
$<BUILD_INTERFACE:${MCUX_BLUETOOTH_DIR}/controller/lib>
$<BUILD_INTERFACE:${MCUX_BLUETOOTH_DIR}/hci_transport/interface>
$<BUILD_INTERFACE:${MCUX_BLUETOOTH_DIR}/host/config>
$<BUILD_INTERFACE:${MCUX_BLUETOOTH_DIR}/host/interface>
$<BUILD_INTERFACE:${MCUX_BLUETOOTH_DIR}/profiles/battery>
$<BUILD_INTERFACE:${MCUX_BLUETOOTH_DIR}/profiles/device_info>
$<BUILD_INTERFACE:${MCUX_BLUETOOTH_DIR}/profiles/temperature>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/SerialManager/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/LED/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Keyboard/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Common>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/OSAbstraction/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/MemManager/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Lists>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/TimersManager/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/RNG/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/MWSCoexistence/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Messaging/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/ModuleInfo>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Panic/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/SecLib>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/TimersManager/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Flash/Internal>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/FunctionLib>
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/devices/MKW41Z4/drivers>
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/devices/MKW41Z4>
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/CMSIS/Include>
$<BUILD_INTERFACE:${TARGET_DIR}/src>
)

if(FRAMEWORK_ENABLED AND FRAMEWORK_KEYBOARD_ENABLED)
  target_sources(${TARGET} PUBLIC
  "${MCUX_FRAMEWORK_DIR}/Keyboard/Source/Keyboard.c"
  )
endif(FRAMEWORK_ENABLED AND FRAMEWORK_KEYBOARD_ENABLED)

if(FRAMEWORK_ENABLED AND FRAMEWORK_SHELL_ENABLED)
  target_sources(${TARGET} PUBLIC
  "${MCUX_FRAMEWORK_DIR}/Shell/Source/shell.c"
  "${MCUX_FRAMEWORK_DIR}/Shell/Source/shell_autocomplete.c"
  "${MCUX_FRAMEWORK_DIR}/Shell/Source/shell_cmdhelp.c"
  "${MCUX_FRAMEWORK_DIR}/Shell/Source/shell_cmdhist.c"
  )
endif(FRAMEWORK_ENABLED AND FRAMEWORK_SHELL_ENABLED)

if(FRAMEWORK_ENABLED)
  target_sources(${TARGET} PUBLIC
  "${MCUX_FRAMEWORK_DIR}/DCDC/Source/DCDC.c"
  "${MCUX_FRAMEWORK_DIR}/NVM/Source/NV_Flash.c"
  "${MCUX_FRAMEWORK_DIR}/LowPower/Source/MKW41Z/PWR.c"
  "${MCUX_FRAMEWORK_DIR}/LowPower/Source/MKW41Z/PWR_BLE.c"
  "${MCUX_FRAMEWORK_DIR}/LowPower/Source/MKW41Z/PWRLib.c"
  "${MCUX_FRAMEWORK_DIR}/Flash/Internal/Flash_Adapter.c"
  "${MCUX_FRAMEWORK_DIR}/GPIO/GPIO_Adapter.c"
  "${MCUX_FRAMEWORK_DIR}/LED/Source/LED.c"
  "${MCUX_FRAMEWORK_DIR}/FunctionLib/FunctionLib.c"
  "${MCUX_FRAMEWORK_DIR}/Lists/GenericList.c"
  "${MCUX_FRAMEWORK_DIR}/MWSCoexistence/Source/MWS.c"
  "${MCUX_FRAMEWORK_DIR}/MemManager/Source/MemManager.c"
  "${MCUX_FRAMEWORK_DIR}/Messaging/Source/Messaging.c"
  "${MCUX_FRAMEWORK_DIR}/ModuleInfo/ModuleInfo.c"
  "${MCUX_FRAMEWORK_DIR}/OSAbstraction/Source/fsl_os_abstraction_free_rtos.c"
  "${MCUX_FRAMEWORK_DIR}/Panic/Source/Panic.c"
  "${MCUX_FRAMEWORK_DIR}/RNG/Source/RNG.c"
  "${MCUX_FRAMEWORK_DIR}/Reset/Reset.c"
  "${MCUX_FRAMEWORK_DIR}/SecLib/SecLib.c"
  "${MCUX_FRAMEWORK_DIR}/SecLib/lib_crypto_m0.a"
  "${MCUX_FRAMEWORK_DIR}/SerialManager/Source/SerialManager.c"
  "${MCUX_FRAMEWORK_DIR}/SerialManager/Source/UART_Adapter.c"
  "${MCUX_FRAMEWORK_DIR}/TimersManager/Source/TMR_Adapter.c"
  "${MCUX_FRAMEWORK_DIR}/TimersManager/Source/TimersManager.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/cfgs_kw4x_3x_2x/fsl_xcvr_ant_config.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/cfgs_kw4x_3x_2x/fsl_xcvr_ble_config.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/cfgs_kw4x_3x_2x/fsl_xcvr_common_config.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/cfgs_kw4x_3x_2x/fsl_xcvr_gfsk_bt_0p3_h_0p5_config.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/cfgs_kw4x_3x_2x/fsl_xcvr_gfsk_bt_0p5_h_0p32_config.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/cfgs_kw4x_3x_2x/fsl_xcvr_gfsk_bt_0p5_h_0p5_config.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/cfgs_kw4x_3x_2x/fsl_xcvr_gfsk_bt_0p5_h_0p7_config.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/cfgs_kw4x_3x_2x/fsl_xcvr_gfsk_bt_0p5_h_1p0_config.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/cfgs_kw4x_3x_2x/fsl_xcvr_gfsk_bt_0p7_h_0p5_config.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/cfgs_kw4x_3x_2x/fsl_xcvr_mode_datarate_config.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/cfgs_kw4x_3x_2x/fsl_xcvr_msk_config.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/cfgs_kw4x_3x_2x/fsl_xcvr_zgbe_config.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/dbg_ram_capture.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/fsl_xcvr.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/fsl_xcvr_trim.c"
  "${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4/ifr_radio.c"
  )
endif(FRAMEWORK_ENABLED)

target_include_directories(${TARGET} PUBLIC
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/CMSIS/Include>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Common>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/DCDC/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/NVM/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Flash/Internal>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/FunctionLib>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/GPIO>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Keyboard/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Shell/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/LED/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Lists>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/MWSCoexistence/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/MemManager/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Messaging/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/ModuleInfo>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/OSAbstraction/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Panic/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/RNG/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/SecLib>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/SerialManager/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/SerialManager/Source>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/TimersManager/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/TimersManager/Source>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/XCVR/MKW41Z4>
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/devices/MKW41Z4/drivers>
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/devices/MKW41Z4>
$<BUILD_INTERFACE:${MCUX_FREERTOS_DIR}/Source/portable/GCC/ARM_CM0>
$<BUILD_INTERFACE:${MCUX_FREERTOS_DIR}/Source/include>
)

if(FREERTOS_ENABLED)
  target_sources(${TARGET} PUBLIC
  "${MCUX_FREERTOS_DIR}/Source/portable/GCC/ARM_CM0/port.c"
  "${MCUX_FREERTOS_DIR}/Source/portable/GCC/ARM_CM0/fsl_tickless_systick.c"
  "${MCUX_FREERTOS_DIR}/Source/portable/MemMang/heap_4.c"
  "${MCUX_FREERTOS_DIR}/Source/croutine.c"
  "${MCUX_FREERTOS_DIR}/Source/list.c"
  "${MCUX_FREERTOS_DIR}/Source/queue.c"
  "${MCUX_FREERTOS_DIR}/Source/tasks.c"
  "${MCUX_FREERTOS_DIR}/Source/timers.c"
  "${MCUX_FREERTOS_DIR}/Source/event_groups.c"
  )
endif(FREERTOS_ENABLED)

target_include_directories(${TARGET} PUBLIC
$<BUILD_INTERFACE:${MCUX_FREERTOS_DIR}/Source/portable/GCC/ARM_CM0>
$<BUILD_INTERFACE:${MCUX_FREERTOS_DIR}/Source/include>
$<BUILD_INTERFACE:${MCUX_FREERTOS_DIR}/Source>
)

if(FREERTOS_ENABLED)
  target_sources(${TARGET} PUBLIC
  "${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_lpuart_freertos.c"
  "${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_dspi_freertos.c"
  "${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_i2c_freertos.c"
  )
endif(FREERTOS_ENABLED)

target_sources(${TARGET} PUBLIC
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_lpuart.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_lptmr.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_common.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/utilities/fsl_debug_console.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/utilities/fsl_notifier.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/system_MKW41Z4.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_gpio.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_smc.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_flash.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_clock.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_adc16.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_dspi.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_i2c.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_llwu.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_ltc.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_pmc.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_rtc.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_tpm.c"
"${MCUX_SDK_DIR}/devices/MKW41Z4/drivers/fsl_trng.c"
)

target_include_directories(${TARGET} PUBLIC
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/CMSIS/Include>
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/devices/MKW41Z4/drivers>
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/devices/MKW41Z4/utilities>
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/devices/MKW41Z4>
)

target_sources(${TARGET} PUBLIC
"${MCUX_SDK_DIR}/devices/MKW41Z4/gcc/startup_MKW41Z4.S"
)

target_include_directories(${TARGET} PUBLIC
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/devices/MKW41Z4/drivers>
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/devices/MKW41Z4/utilities>
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/devices/MKW41Z4>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/GPIO>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Common>
$<BUILD_INTERFACE:${MCUX_SDK_DIR}/CMSIS/Include>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/RNG/Interface>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Flash/Internal>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/FunctionLib>
$<BUILD_INTERFACE:${MCUX_FRAMEWORK_DIR}/Keyboard/Interface>
)

