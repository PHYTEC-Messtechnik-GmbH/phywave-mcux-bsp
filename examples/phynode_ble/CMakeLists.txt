cmake_minimum_required(VERSION 3.0.0)

set(NXP_SOC MKW41Z512xxx4)
set(FLASH_START 0x00000000)
set(TARGET app)

set(GCC_TOOLCHAIN_DIR $ENV{GCC_TOOLCHAIN_DIR})
set(MCUX_SDK_DIR $ENV{MCU_SDK_KW41Z_DIR})
set(JLINK_DIR $ENV{JLINK_DIR})
set(PHYWAVE_SDK_DIR $ENV{PHYWAVE_SDK_DIR})
set(TARGET_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(MCUX_FRAMEWORK_VERSION 5.3.3)
set(MCUX_BLUETOOTH_VERSION 1.2.3)
set(MCUX_FREERTOS_VERSION 9.0.0)
set(MCUX_MIDDLEWARE_LIBS "")
set(BLUETOOTH_ENABLED ON)
set(BLUETOOTH_GATT_ENABLED ON)
set(FRAMEWORK_ENABLED ON)
set(FRAMEWORK_KEYBOARD_ENABLED ON)
set(FREERTOS_ENABLED ON)

include(${PHYWAVE_SDK_DIR}/cmake/armgcc.cmake)

project(${TARGET} C CXX ASM)

if(NOT CMAKE_BUILD_TYPE)
set(CMAKE_BUILD_TYPE DEBUG)
endif()

set(CMAKE_C_FLAGS "-include app_preinclude.h -include FreeRTOSConfig.h")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DFRDM_KW41Z -DFREEDOM")

include(${PHYWAVE_SDK_DIR}/cmake/nxp-kw-base.cmake)

message(STATUS "BUILD_TYPE: " ${CMAKE_BUILD_TYPE})
message(STATUS "CMAKE_OBJSIZE: " ${CMAKE_OBJSIZE})
message(STATUS "CMAKE_OBJCOPY: " ${CMAKE_OBJCOPY})
message(STATUS "CMAKE_DEBUGGER: " ${CMAKE_DEBUGGER})
message(STATUS "CMAKE_ASM_FLAGS_DEBUG: " ${CMAKE_ASM_FLAGS_DEBUG})
message(STATUS "CMAKE_C_FLAGS: " ${CMAKE_C_FLAGS})
message(STATUS "CMAKE_C_FLAGS_DEBUG: " ${CMAKE_C_FLAGS_DEBUG})
message(STATUS "CMAKE_CXX_FLAGS_DEBUG: " ${CMAKE_CXX_FLAGS_DEBUG})
message(STATUS "CMAKE_EXE_LINKER_FLAGS: " ${CMAKE_EXE_LINKER_FLAGS})

include_directories(${TARGET_DIR}/src)

add_executable(${TARGET}
"${TARGET_DIR}/src/app.c"
)

include(${PHYWAVE_SDK_DIR}/cmake/framework.cmake)
include_directories(${PHYWAVE_SDK_DIR}/drivers/pn-athena)
include_directories(${PHYWAVE_SDK_DIR}/drivers/ssd1673)
include_directories(${PHYWAVE_SDK_DIR}/drivers/tcs37727)
include_directories(${PHYWAVE_SDK_DIR}/drivers/hdc1010)
include_directories(${PHYWAVE_SDK_DIR}/drivers/ccs811)
include_directories(${PHYWAVE_SDK_DIR}/drivers/spi)
include_directories(${PHYWAVE_SDK_DIR}/drivers/i2c)

target_sources(${TARGET} PUBLIC
"${TARGET_DIR}/src/app_config.c"
"${PHYWAVE_SDK_DIR}/drivers/pn-athena/gpio_pins.c"
"${PHYWAVE_SDK_DIR}/drivers/pn-athena/pin_mux.c"
"${PHYWAVE_SDK_DIR}/drivers/pn-athena/board.c"
"${PHYWAVE_SDK_DIR}/drivers/pn-athena/clock_config.c"
"${PHYWAVE_SDK_DIR}/drivers/pn-athena/adc_soc.c"
"${PHYWAVE_SDK_DIR}/drivers/ssd1673/ssd1673.c"
"${PHYWAVE_SDK_DIR}/drivers/tcs37727/tcs37727.c"
"${PHYWAVE_SDK_DIR}/drivers/hdc1010/hdc1010.c"
"${PHYWAVE_SDK_DIR}/drivers/ccs811/ccs811.c"
"${PHYWAVE_SDK_DIR}/drivers/i2c/i2c_wrapper.c"
)

message(STATUS "used MIDDLEWARE LIBS: " ${MCUX_MIDDLEWARE_LIBS})

target_link_libraries(${TARGET}
${MCUX_MIDDLEWARE_LIBS}
-Wl,--start-group m
-Wl,--end-group
debug c
debug gcc
debug nosys
optimized m
optimized c
optimized gcc
optimized nosys
)

include(${PHYWAVE_SDK_DIR}/cmake/pb-daplink.cmake)
