#
# InvenSense plugin for OSVR
#
cmake_minimum_required(VERSION 3.0.0)
project(com_Sensics_ART)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(osvr REQUIRED)
find_package(jsoncpp REQUIRED)

add_subdirectory(quat)

# Generate our JSON header file
osvr_convert_json(com_Sensics_ART_json
    com_Sensics_ART.json
    "${CMAKE_CURRENT_BINARY_DIR}/com_Sensics_ART_json.h")

# Be able to find our generated header file and dtrack sdk
include_directories("${CMAKE_CURRENT_BINARY_DIR}"
                    "${DTRACK_SDK_INCLUDEDIR}"
                    "${DTRACK_SDK_SRCDIR}")

# Build and install the plugin
osvr_add_plugin(com_Sensics_ART
                com_Sensics_ART.cpp
                DTrackDataTypes.h
                DTrackSDK.hpp
                DTrackSDK.cpp
                DTrackNet.h
                DTrackNet.cpp
                DTrackParse.hpp
                DTrackParse.cpp
                DTrackParser.hpp
                DTrackParser.cpp
                "${CMAKE_CURRENT_BINARY_DIR}/com_Sensics_ART_json.h")

target_link_libraries(com_Sensics_ART
                    osvr::osvrPluginKitCpp
                    osvr::osvrUtil
                    jsoncpp_lib
                    ws2_32
                    quat)

# Install docs, license, sample config
install(FILES
        osvr_server_config.ART.sample.json
        DESTINATION .)