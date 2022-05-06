# Project Dependences Configuration

# Backup and set build type to release
if(NOT MSVC)
    set(CMAKE_BUILD_TYPE_BAK ${CMAKE_BUILD_TYPE})
    set(CMAKE_BUILD_TYPE Release)
endif()

# Include subdirectories
include_directories(${DEPS_PATHS})

# Find other dependences

# Find Eigen
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR}) 


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#find_package(Dynamixel REQUIRED)
find_package(Dynamixel)
include_directories(${DYNAMIXEL_INCLUDE_DIRS})


# Add subdirectory
foreach(DEPS_PATH ${DEPS_PATHS})
    add_subdirectory(${DEPS_PATH})
endforeach()

# Restore origin build type
if(NOT MSVC)
    set(CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE_BAK})
endif()

