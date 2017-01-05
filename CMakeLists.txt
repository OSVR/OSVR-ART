#
# ART DTrackSDK plugin for OSVR
#
cmake_minimum_required(VERSION 3.1.0)
project(com_Sensics_ART)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(osvr REQUIRED)
find_package(JsonCpp REQUIRED)

add_subdirectory(quat)

# Generate our JSON header file
osvr_convert_json(com_Sensics_ART_json
    com_Sensics_ART.json
    "${CMAKE_CURRENT_BINARY_DIR}/com_Sensics_ART_json.h")

set(DTRACK_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/DTrackSDK")
set(DTRACK_SDK_INCLUDE_DIR "${DTRACK_SDK_ROOT}/include")
set(DTRACK_SDK_SOURCES
    "${DTRACK_SDK_INCLUDE_DIR}/DTrackDataTypes.h"
    "${DTRACK_SDK_INCLUDE_DIR}/DTrackSDK.hpp"
    "${DTRACK_SDK_INCLUDE_DIR}/DTrackNet.h"
    "${DTRACK_SDK_INCLUDE_DIR}/DTrackParse.hpp"
    "${DTRACK_SDK_INCLUDE_DIR}/DTrackParser.hpp"
    "${DTRACK_SDK_ROOT}/src/DTrackNet.cpp"
    "${DTRACK_SDK_ROOT}/src/DTrackParse.cpp"
    "${DTRACK_SDK_ROOT}/src/DTrackParser.cpp"
    "${DTRACK_SDK_ROOT}/src/DTrackSDK.cpp"
)

source_group("DTrack_SDK" FILES ${DTRACK_SDK_SOURCES})

# Build and install the plugin
osvr_add_plugin(com_Sensics_ART
    com_Sensics_ART.cpp
    ${DTRACK_SDK_SOURCES}
    "${CMAKE_CURRENT_BINARY_DIR}/com_Sensics_ART_json.h")

# Be able to find our generated header file and dtrack sdk
target_include_directories(com_Sensics_ART
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_BINARY_DIR}"
    "${DTRACK_SDK_INCLUDE_DIR}")
target_link_libraries(com_Sensics_ART
    osvr::osvrPluginKitCpp
    osvr::osvrUtil
    JsonCpp::JsonCpp
    quat)
if(WIN32)
    target_link_libraries(com_Sensics_ART ws2_32)
endif()


# Install docs, license, sample config
install(FILES
        osvr_server_config.ART.sample.json
        DESTINATION .)
