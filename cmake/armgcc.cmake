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

include(CMakeForceCompiler)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

string(REGEX REPLACE "\\\\" "/" GCC_TOOLCHAIN_DIR "${GCC_TOOLCHAIN_DIR}")

find_program(ARM_NONE_EABI_CC arm-none-eabi-gcc
  /usr/bin
  ${GCC_TOOLCHAIN_DIR}/bin
)
if(NOT ARM_NONE_EABI_CC)
  message(FATAL_ERROR "GCC toolchain not found or GCC_TOOLCHAIN_DIR undefined")
endif()
message(STATUS "found ${ARM_NONE_EABI_CC}")

find_program(ARM_NONE_EABI_CXX arm-none-eabi-g++
  /usr/bin
  ${GCC_TOOLCHAIN_DIR}/bin
)
if(NOT ARM_NONE_EABI_CXX)
  message(FATAL_ERROR "GCC toolchain not found or GCC_TOOLCHAIN_DIR undefined")
endif()
message(STATUS "found ${ARM_NONE_EABI_CXX}")

CMAKE_FORCE_C_COMPILER(${ARM_NONE_EABI_CC} GNU)
CMAKE_FORCE_CXX_COMPILER(${ARM_NONE_EABI_CXX} GNU)
set(CMAKE_ASM_COMPILER ${ARM_NONE_EABI_CC})

set(CMAKE_FIND_ROOT_PATH ${GCC_TOOLCHAIN_DIR}/arm-none-eabi ${EXTRA_FIND_PATH})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
