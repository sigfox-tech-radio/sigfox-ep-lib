################################################################################
#
# Copyright (c) 2024, UnaBiz SAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  1 Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  2 Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
#    used to endorse or promote products derived from this software without
#    specific prior written permission.
#
# NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
# THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
# BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
# IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

mark_as_advanced(UNIFDEF PRECOMPIL_DIR)
# specify the precompil files location
set(PRECOMPIL_DIR ${CMAKE_BINARY_DIR}/precompil CACHE STRING "")

#List of precompileInc and precompileSrc files
foreach(X IN LISTS LIB_SOURCES)
    LIST(APPEND PRECOMPIL_LIB_SOURCES "${PRECOMPIL_DIR}/${X}")
endforeach()
foreach(X IN LISTS MANUF_SOURCES)
    LIST(APPEND PRECOMPIL_MANUF_SOURCES "${PRECOMPIL_DIR}/${X}")
endforeach()
foreach(X IN LISTS LIB_HEADERS)
    LIST(APPEND PRECOMPIL_LIB_HEADERS "${PRECOMPIL_DIR}/${X}")
endforeach()

#Custom command Loop for all Sources
foreach(X IN LISTS LIB_SOURCES MANUF_SOURCES)
add_custom_command(
    OUTPUT "${PRECOMPIL_DIR}/${X}"
    DEPENDS ${CMAKE_BINARY_DIR}/undefs_file
    DEPENDS ${CMAKE_BINARY_DIR}/defs_file
    DEPENDS ${LIB_HEADERS}
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/sigfox_ep_flags.h
    DEPENDS ${X}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${PRECOMPIL_DIR}/src/core ${PRECOMPIL_DIR}/src/manuf
    COMMAND unifdef -B -k -x 2 -f ${CMAKE_BINARY_DIR}/undefs_file -f ${CMAKE_BINARY_DIR}/defs_file ${PROJECT_SOURCE_DIR}/${X} > "${PRECOMPIL_DIR}/${X}" 
    VERBATIM
)
endforeach()

#Custom command Loop for all Headers
foreach(X IN LISTS LIB_HEADERS MANUF_HEADERS)
    add_custom_command(
        OUTPUT ${PRECOMPIL_DIR}/${X}
        DEPENDS ${CMAKE_BINARY_DIR}/undefs_file
        DEPENDS ${CMAKE_BINARY_DIR}/defs_file
        DEPENDS ${LIB_HEADERS}
        DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/sigfox_ep_flags.h
        COMMAND ${CMAKE_COMMAND} -E make_directory ${PRECOMPIL_DIR}/inc/core ${PRECOMPIL_DIR}/inc/manuf
        COMMAND unifdef -B -k -x 2 -f ${CMAKE_BINARY_DIR}/undefs_file -f ${CMAKE_BINARY_DIR}/defs_file ${PROJECT_SOURCE_DIR}/${X} > "${PRECOMPIL_DIR}/${X}"
        VERBATIM
    )
endforeach()

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/defs_file
    DEPENDS ${LIB_HEADERS}
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/sigfox_ep_flags.h
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMAND  cc -E -dMM -I ${CMAKE_CURRENT_BINARY_DIR}  ${CMAKE_CURRENT_SOURCE_DIR}/inc/sigfox_types.h  > "${CMAKE_BINARY_DIR}/defs_file.tmp"
    COMMAND  grep -v __ "${CMAKE_BINARY_DIR}/defs_file.tmp" | sed -E "'s/\\(([0-9]+)\\)/\\1/'" | sort -u > "${CMAKE_BINARY_DIR}/defs_file"
    COMMAND  rm "${CMAKE_BINARY_DIR}/defs_file.tmp"
)

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/undefs_file
    DEPENDS ${LIB_HEADERS}
    DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/sigfox_ep_flags.h
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMAND cat ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c ${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.c ${CMAKE_CURRENT_SOURCE_DIR}/src/manuf/*c ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/core/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/manuf/*.h | unifdef -s | sort -u | grep -v __ | sed "s/^/#undef /" >"${CMAKE_BINARY_DIR}/undefs_file"
)

add_custom_target(precompil
    DEPENDS ${CMAKE_BINARY_DIR}/undefs_file
    DEPENDS ${CMAKE_BINARY_DIR}/defs_file
    DEPENDS ${PRECOMPIL_LIB_SOURCES}
    DEPENDS ${PRECOMPIL_LIB_HEADERS}
    DEPENDS ${PRECOMPIL_MANUF_SOURCES}
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/sigfox_ep_flags.h ${PRECOMPIL_DIR}/inc
    VERBATIM
)