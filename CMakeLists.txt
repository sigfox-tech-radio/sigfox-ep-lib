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

cmake_minimum_required(VERSION 3.21)
project(sigfox_ep_lib)

# Define macro to manage the options 
macro(opt TYPE FLAG DEFAULT DESC)
  if(${TYPE} STREQUAL TYPE_BOOL) 
     #option(${FLAG} ${DESC} ${DEFAULT})
     set(${FLAG} ${DEFAULT} CACHE STRING ${DESC})
     set_property(CACHE ${FLAG} PROPERTY STRINGS ON OFF)
  elseif(${TYPE} STREQUAL TYPE_VALUE)
     set(${FLAG} ${DEFAULT} CACHE STRING ${DESC})
  else()
     message(FATAL_ERROR "Only TYPE_BOOL and TYPE_VALUE are supported")
  endif()
  list(APPEND COMPILE_FLAG_LIST ${FLAG}) 
endmacro()

set(LIB_SOURCES
    src/sigfox_ep_api.c
    src/sigfox_error.c
    src/sigfox_rc.c
    src/sigfox_types.c
    src/core/sigfox_crc.c
    src/core/sigfox_ep_bitstream.c
    src/core/sigfox_ep_frequency.c
    src/core/sigfox_tx_control.c
    src/core/TI_aes_128_encr_only.c
)

set(LIB_HEADERS 
    inc/sigfox_ep_lib_version.h
    inc/sigfox_ep_api.h
    inc/sigfox_ep_api_test.h
    inc/sigfox_error.h
    inc/sigfox_rc.h
    inc/sigfox_types.h
    inc/core/sigfox_ep_bitstream.h
    inc/core/sigfox_ep_frequency.h
    inc/core/sigfox_tx_control.h
    inc/core/sigfox_crc.h
    inc/core/TI_aes_128_encr_only.h
    inc/manuf/mcu_api.h
    inc/manuf/rf_api.h
)

set(LIB_PUBLIC_HEADERS
    inc/core/sigfox_ep_bitstream.h
    inc/core/sigfox_ep_frequency.h
    inc/core/sigfox_tx_control.h
    inc/core/sigfox_crc.h
    inc/core/TI_aes_128_encr_only.h
    inc/manuf/rf_api.h
    inc/manuf/mcu_api.h
    inc/sigfox_ep_api.h
    inc/sigfox_ep_api_test.h
    inc/sigfox_error.h
    inc/sigfox_rc.h
    inc/sigfox_types.h
    inc/sigfox_ep_lib_version.h
    ${CMAKE_CURRENT_BINARY_DIR}/sigfox_ep_flags.h
)

set(MANUF_SOURCES
    src/manuf/mcu_api.c
    src/manuf/rf_api.c
)

# Set the path for the static library
set(LIB_LOCATION ${CMAKE_BINARY_DIR}/lib/ CACHE STRING "")

# Set the path for the public header of the library
set(API_LOCATION ${CMAKE_BINARY_DIR}/lib/api CACHE STRING "")

#Option addon RFP contents
if(NOT sigfox_ep_addon_rfp_IS_TOP_LEVEL)
    opt(TYPE_BOOL ADDON_RFP OFF "Add RFP addon contents to build it with library")
endif()
#Option addon TA contents
if(NOT sigfox_ep_addon_ta_IS_TOP_LEVEL)
    opt(TYPE_BOOL ADDON_TA OFF "Add TA addon contents to build it with library")
endif()
#Option RF API contents
if(NOT s2lp_rf_api_IS_TOP_LEVEL)
    opt(TYPE_BOOL S2LP_RF_API OFF "Add S2LP RF API contents to build it with library")
    if(S2LP_RF_API)
        list(REMOVE_ITEM MANUF_SOURCES "src/manuf/rf_api.c")
    endif()
else()
    list(REMOVE_ITEM MANUF_SOURCES "src/manuf/rf_api.c")
endif()
if(NOT lr11xx_rf_api_IS_TOP_LEVEL)
    opt(TYPE_BOOL LR11XX_RF_API OFF "Add LR11XX RF API contents to build it with library")
    if(LR11XX_RF_API)
        list(REMOVE_ITEM MANUF_SOURCES "src/manuf/rf_api.c")
    endif()
else()
    list(REMOVE_ITEM MANUF_SOURCES "src/manuf/rf_api.c")
endif()
if(NOT sx126x_rf_api_IS_TOP_LEVEL)
    opt(TYPE_BOOL SX126X_RF_API OFF "Add SX126X RF API contents to build it with library")
    if(SX126X_RF_API)
        list(REMOVE_ITEM MANUF_SOURCES "src/manuf/rf_api.c")
    endif()
else()
    list(REMOVE_ITEM MANUF_SOURCES "src/manuf/rf_api.c")
endif()

opt(TYPE_BOOL   SIGFOX_EP_RC1_ZONE                      ON  "Support RC1 (Europe, Middle-East and Africa)")
opt(TYPE_BOOL   SIGFOX_EP_RC2_ZONE                      ON  "Support RC2 (Brazil, Canada, Mexico, Puerto Rico and USA)")
opt(TYPE_BOOL   SIGFOX_EP_RC3_LBT_ZONE                  ON  "Support RC3 with LBT (Japan)")
opt(TYPE_BOOL   SIGFOX_EP_RC3_LDC_ZONE                  ON  "Support RC3 with LDC (Japan)")
opt(TYPE_BOOL   SIGFOX_EP_RC4_ZONE                      ON  "Support RC4 (Latin America and Asia Pacific)")
opt(TYPE_BOOL   SIGFOX_EP_RC5_ZONE                      ON  "Support RC5 (South-Corea)")
opt(TYPE_BOOL   SIGFOX_EP_RC6_ZONE                      ON  "Support RC6 (India)")
opt(TYPE_BOOL   SIGFOX_EP_RC7_ZONE                      ON  "Support RC7 (Russia)")
opt(TYPE_BOOL   SIGFOX_EP_APPLICATION_MESSAGES          ON  "Support uplink application messages")
opt(TYPE_BOOL   SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE    ON  "Support uplink control keep alive message")
opt(TYPE_BOOL   SIGFOX_EP_BIDIRECTIONAL                 ON  "Support downlink communication")
opt(TYPE_BOOL   SIGFOX_EP_ASYNCHRONOUS                  ON  "Support Asynchronous mode")
opt(TYPE_BOOL   SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE          ON  "Enable MCU and RF open/close functions")
opt(TYPE_BOOL   SIGFOX_EP_REGULATORY                    ON  "Enable Regulatory before transmission (DC, FH or LBT)")
opt(TYPE_BOOL   SIGFOX_EP_LATENCY_COMPENSATION          ON  "Enable radio latency compensation to improve MCU timers accuracy")
opt(TYPE_BOOL   SIGFOX_EP_SINGLE_FRAME                  OFF "Send only 1 frame per message (N=1)")
opt(TYPE_BOOL   SIGFOX_EP_PARAMETERS_CHECK              ON  "Enable parameters check")
opt(TYPE_BOOL   SIGFOX_EP_CERTIFICATION                 ON  "Enable certification functions")
opt(TYPE_BOOL   SIGFOX_EP_PUBLIC_KEY_CAPABLE            ON  "Enable public key switch feature")
opt(TYPE_BOOL   SIGFOX_EP_VERBOSE                       ON  "Enable versionning functions")
opt(TYPE_BOOL   SIGFOX_EP_AES_HW                        ON  "Support hardware AES")
opt(TYPE_BOOL   SIGFOX_EP_CRC_HW                        OFF "Support hardware CRC")
opt(TYPE_BOOL   SIGFOX_EP_ERROR_CODES                   ON  "Enable error codes on all functions")
opt(TYPE_VALUE  SIGFOX_EP_UL_BIT_RATE_BPS               OFF "Fixed uplink bit rate in bps (100/600)")
set_property(CACHE SIGFOX_EP_UL_BIT_RATE_BPS PROPERTY STRINGS OFF 100 600)
opt(TYPE_VALUE  SIGFOX_EP_T_IFU_MS                      OFF "Fixed inter-frame delay in ms (10 to 2000)")
opt(TYPE_VALUE  SIGFOX_EP_T_CONF_MS                     OFF "Fixed DL confirmation delay in ms (1400 to 4000)")
opt(TYPE_VALUE  SIGFOX_EP_UL_PAYLOAD_SIZE               OFF "Fixed UL payload size in bytes (0 to 12)")
set_property(CACHE SIGFOX_EP_UL_PAYLOAD_SIZE PROPERTY STRINGS OFF 0 1 2 3 4 5 6 7 8 9 10 11 12)
opt(TYPE_VALUE  SIGFOX_EP_TX_POWER_DBM_EIRP             OFF "Fixed the TX power supported by the radio")
opt(TYPE_VALUE  SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER      OFF "Fixed message counter rollover (128, 256, 512, 1024, 2048 or 4096)")
set_property(CACHE SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER PROPERTY STRINGS OFF 128 256 512 1024 2048 4096)
opt(TYPE_VALUE  SIGFOX_EP_ERROR_STACK                   32  "Enable error stack and defined the depth")

#workaround to force a define to be set to 0 when value is 0 and not define when value is OFF
foreach( COMPILE_FLAG ${COMPILE_FLAG_LIST} )
    if((NOT ${COMPILE_FLAG} STREQUAL OFF) AND (NOT ${COMPILE_FLAG} STREQUAL ON))
        set(${COMPILE_FLAG} "(${${COMPILE_FLAG}})")
    endif()
endforeach()    

if(${SIGFOX_EP_CRC_HW} STREQUAL ON)
    list(REMOVE_ITEM LIB_SOURCES "src/core/sigfox_crc.c")
    list(REMOVE_ITEM LIB_HEADERS "inc/core/sigfox_crc.h")
    list(REMOVE_ITEM LIB_PUBLIC_HEADERS "inc/core/sigfox_crc.h")
endif()
if(${SIGFOX_EP_AES_HW} STREQUAL ON)
    list(REMOVE_ITEM LIB_SOURCES "src/core/TI_aes_128_encr_only.c")
    list(REMOVE_ITEM LIB_HEADERS "inc/core/TI_aes_128_encr_only.h.h")
    list(REMOVE_ITEM LIB_PUBLIC_HEADERS "inc/core/TI_aes_128_encr_only.h")
endif()
if(${SIGFOX_EP_REGULATORY} STREQUAL OFF)
    list(REMOVE_ITEM LIB_SOURCES "src/core/sigfox_tx_control.c")
    list(REMOVE_ITEM LIB_HEADERS "inc/core/sigfox_tx_control.h")
    list(REMOVE_ITEM LIB_PUBLIC_HEADERS "inc/core/sigfox_tx_control.h")
endif()
if(${SIGFOX_EP_CERTIFICATION} STREQUAL OFF)
    list(REMOVE_ITEM LIB_HEADERS "inc/sigfox_ep_api_test.h")
    list(REMOVE_ITEM LIB_PUBLIC_HEADERS "inc/sigfox_ep_api_test.h")
endif()

#Add Cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

#Configure and create sigfox_ep_flags.h
configure_file(sigfox_ep_flags.h.in sigfox_ep_flags.h @ONLY)

#Precompile module
find_program(UNIFDEF unifdef)
if(NOT UNIFDEF)
    message(STATUS "unifdef - not found (precompilation target will not be available)")
else()
    include(precompile)
endif()
#Addon RFP module
if(ADDON_RFP)
    include(addon_rfp)
endif()
#Addon TA module
if(ADDON_TA)
    include(addon_ta)
endif()
#RF_API modules
if(S2LP_RF_API)
    include(s2lp_rf_api)
endif()
if(LR11XX_RF_API)
    include(lr11xx_rf_api)
endif()
if(SX126X_RF_API)
    include(sx126x_rf_api)
endif()
mark_as_advanced(CMAKE_BUILD_TYPE CMAKE_INSTALL_PREFIX LIB_LOCATION API_LOCATION)

#Target to create object
add_library(${PROJECT_NAME}_obj OBJECT EXCLUDE_FROM_ALL ${LIB_SOURCES})
target_include_directories( ${PROJECT_NAME}_obj PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_BINARY_DIR}  
)

#Target to create library
add_library(${PROJECT_NAME} STATIC $<TARGET_OBJECTS:${PROJECT_NAME}_obj>)
target_include_directories(${PROJECT_NAME} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_CURRENT_BINARY_DIR}   
)
set_target_properties(${PROJECT_NAME} PROPERTIES
    PUBLIC_HEADER "${LIB_PUBLIC_HEADERS}"
    ARCHIVE_OUTPUT_DIRECTORY ${LIB_LOCATION}
    LIBRARY_OUTPUT_DIRECTORY ${LIB_LOCATION}
)
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${LIB_LOCATION}
    PUBLIC_HEADER DESTINATION ${API_LOCATION}  
)
