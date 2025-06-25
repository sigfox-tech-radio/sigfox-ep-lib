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

include(ExternalProject)
include(FetchContent)
Set(FETCHCONTENT_QUIET FALSE)
FetchContent_Declare(
    addon_ta
    GIT_REPOSITORY "https://github.com/sigfox-tech-radio/sigfox-ep-addon-ta"
    GIT_TAG "v2.1"
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

