include(ExternalProject)
include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)
FetchContent_Declare(
    sx126x_rf_api
    GIT_REPOSITORY "https://github.com/sigfox-tech-radio/sigfox-ep-rf-api-semtech-sx126x"
    GIT_TAG "v1.2"
    GIT_PROGRESS TRUE
    GIT_SHALLOW    1
    #SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/addons/rfp
    UPDATE_DISCONNECTED TRUE
    STEP_TARGETS        update
)
FetchContent_GetProperties(sx126x_rf_api)
if (NOT platform_POPULATED)
    FetchContent_Populate(sx126x_rf_api)
    add_subdirectory(${sx126x_rf_api_SOURCE_DIR} ${sx126x_rf_api_BINARY_DIR})
endif()
mark_as_advanced(FETCHCONTENT_QUIET FETCHCONTENT_BASE_DIR FETCHCONTENT_FULLY_DISCONNECTED FETCHCONTENT_UPDATES_DISCONNECTED)
mark_as_advanced(FETCHCONTENT_SOURCE_DIR_S_RF_API FETCHCONTENT_UPDATES_DISCONNECTED_SX126X_RF_API)
#FetchContent_MakeAvailable(addon_rfp)
