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
    inc/manuf/mcu_api.h
    inc/manuf/rf_api.h
)

set(LIB_PUBLIC_HEADERS
    inc/core/sigfox_ep_bitstream.h
    inc/core/sigfox_ep_frequency.h
    inc/core/sigfox_tx_control.h
    inc/core/sigfox_crc.h
    inc/manuf/rf_api.h
    inc/manuf/mcu_api.h
    inc/sigfox_ep_api.h
    inc/sigfox_ep_api_test.h
    inc/sigfox_error.h
    inc/sigfox_rc.h
    inc/sigfox_types.h
    inc/sigfox_ep_lib_version.h    
)

set(MANUF_SOURCES
    src/manuf/mcu_api.c
    src/manuf/rf_api.c
)

# Set the path for the static library
set(LIB_LOCATION ${CMAKE_BINARY_DIR}/lib/ CACHE STRING 	"")

# Set the path for the public header of the library
set(API_LOCATION ${CMAKE_BINARY_DIR}/lib/api CACHE STRING 	"")

#Options Use sigfox_ep_flag.h
opt(TYPE_BOOL USE_SIGFOX_EP_FLAGS_H ON "library compilation options. ON:in sigfox_ep_flag.h file OFF:in command line")
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

#When sigfox_ep_flag.h is don't used
if(${USE_SIGFOX_EP_FLAGS_H} STREQUAL "ON")
	list(APPEND DEF_FLAG_LIST "-DUSE_SIGFOX_EP_FLAGS_H")
    list(APPEND LIB_HEADERS 	"inc/sigfox_ep_flags.h")
    unset(RC1_ZONE CACHE)
    unset(RC2_ZONE CACHE)
    unset(RC3C_ZONE CACHE)
    unset(RC3D_ZONE CACHE)
    unset(RC4_ZONE CACHE)
    unset(RC5_ZONE CACHE)
    unset(RC6_ZONE CACHE)
    unset(RC7_ZONE CACHE)
    unset(APPLICATION_MESSAGES CACHE)
    unset(CONTROL_KEEP_ALIVE_MESSAGE CACHE)
    unset(BIDIRECTIONAL CACHE)
    unset(ASYNCHRONOUS CACHE)
    unset(LOW_LEVEL_OPEN_CLOSE CACHE)
    unset(REGULATORY CACHE)
    unset(LATENCY_COMPENSATION CACHE)
    unset(SINGLE_FRAME CACHE)
    unset(PARAMETERS_CHECK CACHE)
    unset(CERTIFICATION CACHE)
    unset(PUBLIC_KEY_CAPABLE CACHE)
    unset(VERBOSE CACHE)
    unset(CRC_HW CACHE)
    unset(ERROR_CODES CACHE)
    unset(UL_BIT_RATE_BPS CACHE)
    unset(T_IFU_MS CACHE)
    unset(T_CONF_MS CACHE)
    unset(UL_PAYLOAD_SIZE CACHE)
    unset(TX_POWER_DBM_EIRP CACHE)
    unset(UL_PAYLOAD_SIZE CACHE)
    unset(MESSAGE_COUNTER_ROLLOVER CACHE)
    unset(ERROR_STACK CACHE)
else()
	opt(TYPE_BOOL   RC1_ZONE                    ON  "Support RC1 (Europe, Middle-East and Africa)")
	opt(TYPE_BOOL   RC2_ZONE                    ON  "Support RC2 (Brazil, Canada, Mexico, Puerto Rico and USA)")
	opt(TYPE_BOOL   RC3C_ZONE                   ON  "Support RC3C with LBT (Japan)")
	opt(TYPE_BOOL   RC3D_ZONE                   ON  "Support RC3D with DC (Japan)")
	opt(TYPE_BOOL   RC4_ZONE                    ON  "Support RC4 (Latin America and Asia Pacific)")
	opt(TYPE_BOOL   RC5_ZONE                    ON  "Support RC5 (South-Corea)")
	opt(TYPE_BOOL   RC6_ZONE                    ON  "Support RC6 (India)")
	opt(TYPE_BOOL   RC7_ZONE                    ON  "Support RC7 (Russia)")
	opt(TYPE_BOOL   APPLICATION_MESSAGES 		ON  "Support uplink application messages")
	opt(TYPE_BOOL   CONTROL_KEEP_ALIVE_MESSAGE	ON  "Support uplink control keep alive message")
	opt(TYPE_BOOL   BIDIRECTIONAL				ON  "Support downlink communication")
	opt(TYPE_BOOL   ASYNCHRONOUS				ON	"Support Asynchronous mode")
	opt(TYPE_BOOL   LOW_LEVEL_OPEN_CLOSE	    ON	"Enable MCU and RF open/close functions")
	opt(TYPE_BOOL   REGULATORY					ON  "Enable Regulatory before transmission (DC, FH or LBT)")
	opt(TYPE_BOOL   LATENCY_COMPENSATION		ON  "Enable radio latency compensation to improve MCU timers accuracy")
	opt(TYPE_BOOL   SINGLE_FRAME			    OFF	"Send only 1 frame per message (N=1)")
	opt(TYPE_BOOL   PARAMETERS_CHECK		    ON	"Enable parameters check")
	opt(TYPE_BOOL   CERTIFICATION				ON  "Enable certification functions")
	opt(TYPE_BOOL   PUBLIC_KEY_CAPABLE			ON  "Enable public key switch feature")
	opt(TYPE_BOOL   VERBOSE						ON  "Enable versionning functions")
	opt(TYPE_BOOL   CRC_HW						OFF "Support hardware CRC")
	opt(TYPE_BOOL   ERROR_CODES					ON  "Enable error codes on all functions")
	opt(TYPE_VALUE  UL_BIT_RATE_BPS             OFF "Fixed uplink bit rate in bps (100/600)")
	set_property(CACHE UL_BIT_RATE_BPS PROPERTY STRINGS OFF 100 600)
	opt(TYPE_VALUE  T_IFU_MS					OFF "Fixed inter-frame delay in ms (10 to 2000)")
	opt(TYPE_VALUE  T_CONF_MS					OFF "Fixed DL confirmation delay in ms (1400 to 4000)")
	opt(TYPE_VALUE  UL_PAYLOAD_SIZE 			OFF "Fixed UL payload size in bytes (0 to 12)")
	set_property(CACHE UL_PAYLOAD_SIZE PROPERTY STRINGS OFF 0 1 2 3 4 5 6 7 8 9 10 11 12)
	opt(TYPE_VALUE  TX_POWER_DBM_EIRP           OFF "Fixed the TX power supported by the radio")
	opt(TYPE_VALUE  MESSAGE_COUNTER_ROLLOVER 	OFF "Fixed message counter rollover (128, 256, 512, 1024, 2048 or 4096)")
	set_property(CACHE MESSAGE_COUNTER_ROLLOVER PROPERTY STRINGS OFF 128 256 512 1024 2048 4096)
	opt(TYPE_VALUE  ERROR_STACK 				32  "Enable error stack and defined the depth")

	foreach( COMPILE_FLAG ${COMPILE_FLAG_LIST} )
	    if((NOT ${COMPILE_FLAG} STREQUAL OFF))
	    	if(${${COMPILE_FLAG}} STREQUAL ON)
	    		 list(APPEND DEF_FLAG_LIST "-D${COMPILE_FLAG}")
	    	else()
	    		list(APPEND DEF_FLAG_LIST "-D${COMPILE_FLAG}=${${COMPILE_FLAG}} ")
	    		list(APPEND DEF_FLAG_WITH_VALUE_LIST "${COMPILE_FLAG}")
	    	endif()   		
		endif()
	endforeach()
    if(${CRC_HW} STREQUAL ON)
        list(REMOVE_ITEM LIB_SOURCES "src/core/sigfox_crc.c")
        list(REMOVE_ITEM LIB_HEADERS "inc/core/sigfox_crc.h")
        list(REMOVE_ITEM LIB_PUBLIC_HEADERS "inc/core/sigfox_crc.h")
    endif()
    if(${REGULATORY} STREQUAL OFF)
        list(REMOVE_ITEM LIB_SOURCES "src/core/sigfox_tx_control.c")
        list(REMOVE_ITEM LIB_HEADERS "inc/core/sigfox_tx_control.h")
        list(REMOVE_ITEM LIB_PUBLIC_HEADERS "inc/core/sigfox_tx_control.h")
    endif()
    if(${CERTIFICATION} STREQUAL OFF)
        list(REMOVE_ITEM LIB_HEADERS "inc/sigfox_ep_api_test.h")
        list(REMOVE_ITEM LIB_PUBLIC_HEADERS "inc/sigfox_ep_api_test.h")
    endif()
endif()	
#add DEF_FLAG_LIST to parent scope to be used by a child
if(NOT (${CMAKE_PROJECT_NAME} STREQUAL ${PROJECT_NAME}))
    set(DEF_FLAG_LIST ${DEF_FLAG_LIST}  PARENT_SCOPE)
endif()

#Add Cmake module path
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

#Precompile module
include(precompile)
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

add_library(${PROJECT_NAME}_obj OBJECT EXCLUDE_FROM_ALL ${LIB_SOURCES})
target_include_directories( ${PROJECT_NAME}_obj PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)
target_compile_definitions(${PROJECT_NAME}_obj PUBLIC ${DEF_FLAG_LIST})

add_library(${PROJECT_NAME} STATIC ${PRECOMPIL_LIB_SOURCES})
target_include_directories(${PROJECT_NAME} PUBLIC ${PRECOMPIL_DIR}/inc)
target_compile_definitions(${PROJECT_NAME} PUBLIC ${DEF_FLAG_LIST})
target_compile_options(${PROJECT_NAME} PRIVATE -Wall)
set_target_properties(${PROJECT_NAME} PROPERTIES PUBLIC_HEADER "${PRECOMPIL_LIB_PUBLIC_HEADERS}")
set_target_properties(${PROJECT_NAME}
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${LIB_LOCATION}
    LIBRARY_OUTPUT_DIRECTORY ${LIB_LOCATION}
)
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${LIB_LOCATION}
    PUBLIC_HEADER DESTINATION ${API_LOCATION}  
)
