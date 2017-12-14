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

if(NOT DEFINED JLINK_DIR)
  message(FATAL_ERROR "JLink location undefined")
endif(NOT DEFINED JLINK_DIR)

if(NOT DEFINED JLINK_DEVICE)
  message(FATAL_ERROR "JLINK_DEVICE undefined")
endif(NOT DEFINED JLINK_DEVICE)

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
configure_file(${CMAKE_CURRENT_LIST_DIR}/flash.jcmd ${EXECUTABLE_OUTPUT_PATH}/flash.jlink)
configure_file(${CMAKE_CURRENT_LIST_DIR}/erase.jcmd ${EXECUTABLE_OUTPUT_PATH}/erase.jlink)
configure_file(${CMAKE_CURRENT_LIST_DIR}/reset.jcmd ${EXECUTABLE_OUTPUT_PATH}/reset.jlink)

add_custom_target(debug
  COMMAND ${CMAKE_DEBUGGER} -tui -command ${CMAKE_CURRENT_LIST_DIR}/jlink.gdbconf
          ${EXECUTABLE_OUTPUT_PATH}/${TARGET}.elf
  DEPENDS ${TARGET}
)

add_custom_target(gdb-server
  COMMAND ${JLINK_DIR}/JLinkGDBServer -device ${JLINK_DEVICE} -speed 4000 -if SWD
  DEPENDS ${TARGET}
)

# The documentation of the Commander options can be found on
# https://wiki.segger.com/index.php?title=J-Link_Commander
# or "J-Link / J-Trace User Guide" Chapter 3.

add_custom_target(flash
  COMMAND ${JLINK_DIR}/JLinkExe -device ${JLINK_DEVICE} -speed 4000 -if SWD -CommanderScript
          ${EXECUTABLE_OUTPUT_PATH}/flash.jlink
  DEPENDS ${EXECUTABLE_OUTPUT_PATH}/${TARGET}
)

add_custom_target(erase
  COMMAND ${JLINK_DIR}/JLinkExe -device ${JLINK_DEVICE} -speed 4000 -if SWD -CommanderScript
          ${EXECUTABLE_OUTPUT_PATH}/erase.jlink
)

add_custom_target(reset
  COMMAND ${JLINK_DIR}/JLinkExe -device ${JLINK_DEVICE} -speed 4000 -if SWD -CommanderScript
          ${EXECUTABLE_OUTPUT_PATH}/reset.jlink
)
