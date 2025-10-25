cmake_minimum_required(VERSION 3.16)

project(cMesh)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimize CMake for faster builds
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_FIND_PACKAGE_NO_PACKAGE_REGISTRY ON)
set(CMAKE_FIND_PACKAGE_NO_SYSTEM_PACKAGE_REGISTRY ON)

# Use system packages (only find if not already found)
if(NOT VTK_FOUND)
    find_package(VTK REQUIRED)
endif()
if(NOT Eigen3_FOUND)
    find_package(Eigen3 REQUIRED)
endif()
if(NOT spdlog_FOUND)
    find_package(spdlog REQUIRED)
endif()
if(NOT fmt_FOUND)
    find_package(fmt REQUIRED)
endif()
if(NOT GTest_FOUND)
    find_package(GTest REQUIRED)
endif()

# Add subdirectories
add_subdirectory(src)
add_subdirectory(tests)