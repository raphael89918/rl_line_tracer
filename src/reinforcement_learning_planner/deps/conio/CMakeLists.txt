cmake_minimum_required(VERSION 2.8)

# Environment variables
set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")
set(BUILD_TEST OFF CACHE BOOL "Enable building test")

# Set project
project(conio C CXX)

# Add subdirectory
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

# Build test
if(${BUILD_TEST})
	include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/test)
endif()
