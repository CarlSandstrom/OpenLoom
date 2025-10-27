# CMakeLists.txt for Examples
add_executable(cMeshExample CreateBox.cc)

# Link with all the project modules
target_link_libraries(cMeshExample PRIVATE 
    Geometry
    Topology
    Readers
    Meshing
    Export
)

# Link external libraries
find_package(VTK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

# Find OpenCASCADE for direct linking
set(OpenCASCADE_DIR "$ENV{HOME}/lib/opencascade/lib/cmake/opencascade")
find_package(OpenCASCADE REQUIRED)

target_link_libraries(cMeshExample PRIVATE 
    VTK::CommonCore
    Eigen3::Eigen
    spdlog::spdlog
    fmt::fmt
    # OpenCASCADE libraries
    TKernel
    TKMath
    TKG3d
    TKG2d
    TKGeomBase
    TKBRep
    TKGeomAlgo
    TKTopAlgo
    TKDESTEP
    # TKDESTEPBase
    # TKDESTEPAttr
    # TKDESTEP209
)

# Include OpenCASCADE headers
target_include_directories(cMeshExample PRIVATE 
    ${CMAKE_SOURCE_DIR}/src
    ${OpenCASCADE_INCLUDE_DIR}
)