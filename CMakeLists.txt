cmake_minimum_required(VERSION 3.16)

project(cMesh)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find OpenCASCADE
set(OpenCASCADE_DIR "$ENV{HOME}/lib/opencascade/lib/cmake/opencascade")
find_package(OpenCASCADE REQUIRED)

message(STATUS "OpenCASCADE found: ${OpenCASCADE_INCLUDE_DIR}")
message(STATUS "OpenCASCADE version: ${OpenCASCADE_MAJOR_VERSION}.${OpenCASCADE_MINOR_VERSION}.${OpenCASCADE_MAINTENANCE_VERSION}")

# Use system packages
find_package(VTK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)
find_package(GTest REQUIRED)

# Add subdirectories
add_subdirectory(src)
add_subdirectory(tests)

add_library(meshlib
    # src/geometry_engine.cpp
    # src/mesh_generator.cpp
)

target_include_directories(meshlib
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${OpenCASCADE_INCLUDE_DIR}
)

target_link_libraries(meshlib
    PUBLIC
        TKernel
        TKMath
        TKG3d
        TKG2d
        TKGeomBase
        TKBRep
        TKGeomAlgo
        TKTopAlgo
        TKSTEP
        TKSTEPBase
        TKSTEPAttr
        TKSTEP209
)

# Test executable
add_executable(mesh_test src/main.cpp)
target_link_libraries(mesh_test PRIVATE meshlib)