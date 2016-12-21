cmake_minimum_required(VERSION 3.0)
project(quatlib)

set(QUATLIB_SOURCES matrix.c quat.c vector.c xyzquat.c)
set(QUATLIB_HEADER quat.h)

# Build the library itself and declare what bits need to be installed
add_library(quat ${QUATLIB_SOURCES} ${QUATLIB_HEADER})

set_property(TARGET quat PROPERTY PUBLIC_HEADER ${QUATLIB_HEADER})
set_target_properties(quat PROPERTIES FOLDER Library)

# install(TARGETS
    # quat
    # RUNTIME DESTINATION bin COMPONENT clientsdk
    # LIBRARY DESTINATION lib${LIB_SUFFIX} COMPONENT clientsdk
    # ARCHIVE DESTINATION lib${LIB_SUFFIX} COMPONENT clientsdk
    # PUBLIC_HEADER DESTINATION include COMPONENT clientsdk)
# endif()
