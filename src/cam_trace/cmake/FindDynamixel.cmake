# This will define the following variables::
#
#   Dynamixel_FOUND    - True if the system has the libserialport library
#   DYNAMIXEL_INCLUDE_DIRS     - location of header files
#   DYNAMIXEL_LIBRARIES        - location of library files


find_path(DYNAMIXEL_INCLUDE_DIRS dynamixel_sdk.h
    "/usr/include"
    "/usr/local/include"
    "/usr/local/include/dynamixel_sdk"
    "/home/ical/api/DynamixelSDK/c++/include/dynamixel_sdk"
    )
find_library(DYNAMIXEL_LIBRARIES libdxl_x64_cpp.so
    "/usr/lib"
    "/usr/local/lib"
    "/home/ical/api/DynamixelSDK/c++/build/linux64"
    )
include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(Dynamixel
    FOUND_VAR Dynamixel_FOUND
    REQUIRED_VARS DYNAMIXEL_INCLUDE_DIRS DYNAMIXEL_LIBRARIES
    )


