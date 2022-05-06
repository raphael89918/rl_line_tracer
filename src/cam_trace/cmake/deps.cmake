set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(Dynamixel REQUIRED)
include_directories(${DYNAMIXEL_INCLUDE_DIRS})

foreach(DEPS_PATH ${DEPS_PATH})
    add_subdirectory(${DEPS_PATH})
endforeach()
