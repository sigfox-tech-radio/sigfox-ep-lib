include(ExternalProject)
include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)
FetchContent_Declare(
    addon_rfp
    GIT_REPOSITORY "https://github.com/sigfox-tech-radio/sigfox-ep-addon-rfp"
    GIT_TAG "master"
    GIT_PROGRESS TRUE
    GIT_SHALLOW    1
    #SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/addons/rfp
    UPDATE_DISCONNECTED TRUE
    STEP_TARGETS        update
)
FetchContent_GetProperties(addon_rfp)
if (NOT platform_POPULATED)
    FetchContent_Populate(addon_rfp)
    add_subdirectory(${addon_rfp_SOURCE_DIR} ${addon_rfp_BINARY_DIR})
endif()
#FetchContent_MakeAvailable(addon_rfp)

