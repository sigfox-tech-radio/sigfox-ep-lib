include(ExternalProject)
include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)
FetchContent_Declare(
    addon_ta
    GIT_REPOSITORY "https://github.com/sigfox-tech-radio/sigfox-ep-addon-ta"
    GIT_TAG "v1.0"
    GIT_PROGRESS TRUE
    GIT_SHALLOW    1
    #SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/addons/ta
    UPDATE_DISCONNECTED TRUE
    STEP_TARGETS        update
)
FetchContent_GetProperties(addon_ta)
if (NOT platform_POPULATED)
    FetchContent_Populate(addon_ta)
    add_subdirectory(${addon_ta_SOURCE_DIR} ${addon_ta_BINARY_DIR})
endif()
mark_as_advanced(FETCHCONTENT_QUIET FETCHCONTENT_BASE_DIR FETCHCONTENT_FULLY_DISCONNECTED FETCHCONTENT_UPDATES_DISCONNECTED)
mark_as_advanced(FETCHCONTENT_SOURCE_DIR_ADDON_TA FETCHCONTENT_UPDATES_DISCONNECTED_ADDON_TA)
#FetchContent_MakeAvailable(addon_ta)

