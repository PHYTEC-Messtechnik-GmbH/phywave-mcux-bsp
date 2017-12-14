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

if(NOT DEFINED PYOCD_TARGET)
  message(FATAL_ERROR "PYOCD_TARGET undefined")
endif(NOT DEFINED PYOCD_TARGET)

add_custom_command(TARGET ${TARGET} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary ${EXECUTABLE_OUTPUT_PATH}/${TARGET}.elf
          ${EXECUTABLE_OUTPUT_PATH}/${TARGET}.bin
)

add_custom_command(TARGET ${TARGET} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex ${EXECUTABLE_OUTPUT_PATH}/${TARGET}.elf
          ${EXECUTABLE_OUTPUT_PATH}/${TARGET}.hex
)

add_custom_command(TARGET ${TARGET} POST_BUILD
  COMMAND ${CMAKE_OBJSIZE} ${EXECUTABLE_OUTPUT_PATH}/${TARGET}.elf
)

set(BINARY ${TARGET}.bin)

add_custom_target(debug
  COMMAND ${CMAKE_DEBUGGER} -tui -command ${CMAKE_CURRENT_LIST_DIR}/pyocd.gdbconf
          ${EXECUTABLE_OUTPUT_PATH}/${TARGET}.elf
  DEPENDS ${TARGET}
)

add_custom_target(gdb-server
  COMMAND pyocd-gdbserver --target ${PYOCD_TARGET} --frequency 4000000
  DEPENDS ${TARGET}
)

add_custom_target(flash
  COMMAND pyocd-flashtool --target ${PYOCD_TARGET} --chip_erase --frequency 4000000
          ${EXECUTABLE_OUTPUT_PATH}/${TARGET}.bin
  DEPENDS ${EXECUTABLE_OUTPUT_PATH}/${TARGET}
)

add_custom_target(erase
  COMMAND pyocd-flashtool --target ${PYOCD_TARGET} --chip_erase
)
