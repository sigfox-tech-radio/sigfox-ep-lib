
find_program(UNIFDEF unifdef REQUIRED)
if(NOT UNIFDEF)
    message(FATAL_ERROR "unifdef not found!")
endif()
find_program(SPLINT splint REQUIRED)
if(NOT SPLINT)
    message(FATAL_ERROR "splint not found!")
endif()
mark_as_advanced(SPLINT UNIFDEF PRECOMPIL_DIR)
# specify the precompil files location
set(PRECOMPIL_DIR ${CMAKE_BINARY_DIR}/precompil CACHE STRING 	"")

#List of precompileInc and precompileSrc files
foreach(X IN LISTS LIB_SOURCES)
	LIST(APPEND PRECOMPIL_LIB_SOURCES "${PRECOMPIL_DIR}/${X}")
    LIST(APPEND SPLINT_PRECOMPIL_LIB_SOURCES "${PRECOMPIL_DIR}/splint/${X}")
endforeach()
foreach(X IN LISTS MANUF_SOURCES)
	LIST(APPEND PRECOMPIL_MANUF_SOURCES "${PRECOMPIL_DIR}/${X}")
endforeach()
foreach(X IN LISTS LIB_HEADERS)
	LIST(APPEND PRECOMPIL_LIB_HEADERS "${PRECOMPIL_DIR}/${X}")
endforeach()
foreach(X IN LISTS LIB_PUBLIC_HEADERS)
	LIST(APPEND PRECOMPIL_LIB_PUBLIC_HEADERS "${PRECOMPIL_DIR}/${X}")
endforeach()

#Custom command Loop for all Sources
foreach(X IN LISTS LIB_SOURCES MANUF_SOURCES)
add_custom_command(
	OUTPUT "${PRECOMPIL_DIR}/${X}"
	DEPENDS ${CMAKE_BINARY_DIR}/undefs_file
	DEPENDS ${CMAKE_BINARY_DIR}/defs_file
	DEPENDS ${LIB_HEADERS}
    DEPENDS ${X}
	COMMAND	${CMAKE_COMMAND} -E make_directory ${PRECOMPIL_DIR}/src/core ${PRECOMPIL_DIR}/src/manuf
    COMMAND unifdef -B -k -x 2 -f ${CMAKE_BINARY_DIR}/undefs_file -f ${CMAKE_BINARY_DIR}/defs_file ${PROJECT_SOURCE_DIR}/${X} > "${PRECOMPIL_DIR}/${X}" 
	VERBATIM
)
endforeach()

#Custom command Loop for all Sources : SPLINT analysis
foreach(X IN LISTS LIB_SOURCES MANUF_SOURCES)
add_custom_command(
	OUTPUT "${PRECOMPIL_DIR}/splint/${X}"
	DEPENDS ${CMAKE_BINARY_DIR}/undefs_file
	DEPENDS ${CMAKE_BINARY_DIR}/defs_file
	DEPENDS ${LIB_HEADERS}
    DEPENDS ${X}
	COMMAND	${CMAKE_COMMAND} -E make_directory ${PRECOMPIL_DIR}/splint ${PRECOMPIL_DIR}/splint/src/core ${PRECOMPIL_DIR}/splint/src/manuf
    COMMAND splint  +linelength 150 +weak +show-summary +stats -slashslashcomment -preproc -I${PRECOMPIL_DIR}/inc/core/ -I${PRECOMPIL_DIR}/inc/manuf/ -I${PRECOMPIL_DIR}/inc/  ${PRECOMPIL_DIR}/${X} > ${PRECOMPIL_DIR}/splint/${X}.log
    # Keep a log trace even if the splint failed
    COMMAND cat ${PRECOMPIL_DIR}/splint/${X}.log > ${PRECOMPIL_DIR}/splint/${X}
#	VERBATIM
)
endforeach()


#Custom command Loop for all Headers
foreach(X IN LISTS LIB_HEADERS MANUF_HEADERS)
if(${X} STREQUAL "inc/sigfox_types.h" AND ${USE_SIGFOX_EP_FLAGS_H} STREQUAL "OFF" AND NOT "${DEF_FLAG_WITH_VALUE_LIST}" STREQUAL "")
#Add Specific Macro in sigfox_types.h file
	foreach(X IN LISTS DEF_FLAG_WITH_VALUE_LIST)
		string(CONCAT DEF_FLAG_STRING ${DEF_FLAG_STRING} "\r\\n" "#define ${X} ${${X}}")
	endforeach()
	add_custom_command(
		OUTPUT ${PRECOMPIL_DIR}/${X}
		DEPENDS ${CMAKE_BINARY_DIR}/undefs_file
		DEPENDS ${CMAKE_BINARY_DIR}/defs_file
		DEPENDS ${LIB_HEADERS}
		COMMAND ${CMAKE_COMMAND} -E make_directory ${PRECOMPIL_DIR}/inc/core ${PRECOMPIL_DIR}/inc/manuf
	    COMMAND unifdef -B -k -x 2 -f ${CMAKE_BINARY_DIR}/undefs_file -f ${CMAKE_BINARY_DIR}/defs_file ${PROJECT_SOURCE_DIR}/${X} > "${PRECOMPIL_DIR}/${X}"
		COMMAND sed -i "/SIGFOX library common macros/a ${DEF_FLAG_STRING}" ${PRECOMPIL_DIR}/${X}
		VERBATIM
	)
else()
	add_custom_command(
		OUTPUT ${PRECOMPIL_DIR}/${X}
		DEPENDS ${CMAKE_BINARY_DIR}/undefs_file
		DEPENDS ${CMAKE_BINARY_DIR}/defs_file
		DEPENDS ${LIB_HEADERS}
		COMMAND ${CMAKE_COMMAND} -E make_directory ${PRECOMPIL_DIR}/inc/core ${PRECOMPIL_DIR}/inc/manuf
	    COMMAND unifdef -B -k -x 2 -f ${CMAKE_BINARY_DIR}/undefs_file -f ${CMAKE_BINARY_DIR}/defs_file ${PROJECT_SOURCE_DIR}/${X} > "${PRECOMPIL_DIR}/${X}"
		VERBATIM
	)
endif()
endforeach()

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/defs_file
	DEPENDS ${LIB_HEADERS}
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    COMMAND  cc -E -dMM ${DEF_FLAG_LIST} -I config/ ${CMAKE_CURRENT_SOURCE_DIR}/inc/sigfox_types.h  > "${CMAKE_BINARY_DIR}/defs_file.tmp"
    COMMAND  grep -v __ "${CMAKE_BINARY_DIR}/defs_file.tmp" | sort -u > "${CMAKE_BINARY_DIR}/defs_file"
    COMMAND  rm "${CMAKE_BINARY_DIR}/defs_file.tmp"
)

add_custom_command(OUTPUT ${CMAKE_BINARY_DIR}/undefs_file
	DEPENDS ${LIB_HEADERS}
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	COMMAND cat ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c ${CMAKE_CURRENT_SOURCE_DIR}/src/core/*.c ${CMAKE_CURRENT_SOURCE_DIR}/src/manuf/*c ${CMAKE_CURRENT_SOURCE_DIR}/inc/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/core/*.h ${CMAKE_CURRENT_SOURCE_DIR}/inc/manuf/*.h | unifdef -s | sort -u | grep -v __ | sed "s/^/#undef /" >"${CMAKE_BINARY_DIR}/undefs_file"
)

add_custom_target(precompil
	DEPENDS ${CMAKE_BINARY_DIR}/undefs_file
	DEPENDS ${CMAKE_BINARY_DIR}/defs_file
    DEPENDS ${PRECOMPIL_LIB_SOURCES}
	DEPENDS ${PRECOMPIL_LIB_HEADERS}
	DEPENDS ${PRECOMPIL_MANUF_SOURCES}
	VERBATIM
)

add_custom_target(splintanalysis
	DEPENDS precompil
    DEPENDS ${SPLINT_PRECOMPIL_LIB_SOURCES}
	VERBATIM
)


