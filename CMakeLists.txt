cmake_minimum_required(VERSION 3.16)

# Compiler selection
# To use clang (current):
set(CMAKE_C_COMPILER clang)
set(CMAKE_CXX_COMPILER clang++)
# To switch back to GCC, comment out the clang lines above and uncomment these:
# set(CMAKE_C_COMPILER gcc)
# set(CMAKE_CXX_COMPILER g++)

project(cMesh)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Debug configuration - maximum debug information
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "-g3 -O0 -DDEBUG -fno-omit-frame-pointer")
set(CMAKE_C_FLAGS_DEBUG "-g3 -O0 -DDEBUG -fno-omit-frame-pointer")

# Additional clang-specific debug flags
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fstandalone-debug -glldb")
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -fstandalone-debug -glldb")
endif()

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