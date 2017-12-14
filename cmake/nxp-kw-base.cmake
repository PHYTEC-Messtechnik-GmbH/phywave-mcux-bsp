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

UNSET(CMAKE_C_FLAGS CACHE)
UNSET(CMAKE_CXX_FLAGS CACHE)
UNSET(CMAKE_ASM_FLAGS CACHE)

find_program(ARM_NONE_EABI_SIZE arm-none-eabi-size
  /usr/bin
  ${GCC_TOOLCHAIN_DIR}/bin
)
if(NOT ARM_NONE_EABI_SIZE)
  message(FATAL_ERROR "GCC toolchain not found or GCC_TOOLCHAIN_DIR undefined")
endif()
message(STATUS "found ${ARM_NONE_EABI_SIZE}")

find_program(ARM_NONE_EABI_OBJCOPY arm-none-eabi-objcopy
  /usr/bin
  ${GCC_TOOLCHAIN_DIR}/bin
)
if(NOT ARM_NONE_EABI_OBJCOPY)
  message(FATAL_ERROR "GCC toolchain not found or GCC_TOOLCHAIN_DIR undefined")
endif()
message(STATUS "found ${ARM_NONE_EABI_OBJCOPY}")

find_program(ARM_NONE_EABI_GDB arm-none-eabi-gdb
  /usr/bin
  ${GCC_TOOLCHAIN_DIR}/bin
)
if(NOT ARM_NONE_EABI_GDB)
  message(FATAL_ERROR "GCC toolchain not found or GCC_TOOLCHAIN_DIR undefined")
endif()
message(STATUS "found ${ARM_NONE_EABI_GDB}")

set(CMAKE_OBJSIZE ${ARM_NONE_EABI_SIZE})
set(CMAKE_OBJCOPY ${ARM_NONE_EABI_OBJCOPY})
set(CMAKE_DEBUGGER ${ARM_NONE_EABI_GDB})

set(CMAKE_EXECUTABLE_SUFFIX ".elf")

set(CMAKE_STATIC_LIBRARY_PREFIX)
set(CMAKE_STATIC_LIBRARY_SUFFIX)

set(CMAKE_EXECUTABLE_LIBRARY_PREFIX)
set(CMAKE_EXECUTABLE_LIBRARY_SUFFIX)

#if(CMAKE_BUILD_TYPE MATCHES RELEASE)
#elseif(CMAKE_BUILD_TYPE MATCHES DEBUG)
#endif()

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/output)

add_custom_target(mrproper
  COMMAND ${CMAKE_COMMAND} -P ${PHYWAVE_SDK_DIR}/cmake/mrproper.cmake
)

if(NOT DEFINED NXP_SOC)
  message(FATAL_ERROR "SoC undefined")
endif(NOT DEFINED NXP_SOC)

if(NXP_SOC MATCHES MKW41Z512xxx4)
  set(DEVICE CPU_MKW41Z512VHT4)
  set(JLINK_DEVICE ${NXP_SOC})
  set(PYOCD_TARGET kw41z512xxx4)
elseif()
  message(FATAL_ERROR "unsupportet SoC")
endif()

if(NOT DEFINED FLASH_BASE)
set(FLASH_BASE 0x00000000)
endif(NOT DEFINED FLASH_BASE)

message(STATUS "NXP SOC: " ${NXP_SOC})
message(STATUS "FLASH BASE ADDRESS: " ${FLASH_BASE})
message(STATUS "JLink DEVICE: " ${JLINK_DEVICE})
message(STATUS "pyOCD TARGET: " ${PYOCD_TARGET})

set(CMAKE_ASM_FLAGS "-D__STARTUP_CLEAR_BSS \
-mcpu=cortex-m0plus \
-Wall \
-mfloat-abi=soft \
-mthumb \
-fno-common \
-ffunction-sections \
-fdata-sections \
-ffreestanding \
-fno-builtin \
-mapcs \
-std=gnu99 \
")

set(CMAKE_ASM_FLAGS_DEBUG "${CMAKE_ASM_FLAGS} -DDEBUG -g")
set(CMAKE_ASM_FLAGS_RELEASE "${CMAKE_ASM_FLAGS}")

#add_definitions("-D${DEVICE}")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} \
-DCPU_MKW41Z512VHT4 \
-DFSL_RTOS_FREE_RTOS \
-DPRINTF_FLOAT_ENABLE=1 \
-DPRINTF_ADVANCED_ENABLE=1 \
-mcpu=cortex-m0plus \
-Wall \
-mfloat-abi=soft \
-mthumb \
-MMD \
-MP \
-fno-common \
-ffunction-sections \
-fdata-sections \
-ffreestanding \
-fno-builtin \
-mapcs \
-std=gnu99 \
")

set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS} -DDEBUG -g -O0")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS} -DNDEBUG -Os")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} \
-mcpu=cortex-m0plus \
-Wall \
-mfloat-abi=soft \
-mthumb \
-MMD \
-MP \
-fno-common \
-ffunction-sections \
-fdata-sections \
-ffreestanding \
-fno-builtin \
-mapcs \
-fno-rtti \
-fno-exceptions \
")

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -DDEBUG -g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -Os")

set(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m0plus \
-Wall \
-mfloat-abi=soft \
--specs=nano.specs \
--specs=nosys.specs \
-fno-common \
-ffunction-sections \
-fdata-sections \
-ffreestanding \
-fno-builtin \
-mthumb \
-mapcs \
-Xlinker \
--gc-sections \
-Xlinker \
-static \
-Xlinker \
-z \
-Xlinker \
muldefs \
-T${PHYWAVE_SDK_DIR}/ldscripts/MKW41Z512xxx4_connectivity.ld -static \
")

#set(CMAKE_EXE_LINKER_FLAGS_DEBUG "-g ${CMAKE_EXE_LINKER_FLAGS}")

#set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS}")
