# CMakeLists.txt for 3D Examples

# List all 3D example sources here; targets are created automatically below.
set(EXAMPLE_SOURCES_3D
    CreateBox.cc
    BoxWithHole.cc
    ConstrainedBoxExample.cc
    basic_3d_constraint_example.cpp
    3d_mesh_with_holes_example.cpp
)

# Link external libraries
find_package(VTK REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

# Find OpenCASCADE for direct linking
set(OpenCASCADE_DIR "$ENV{HOME}/lib/opencascade/lib/cmake/opencascade")
find_package(OpenCASCADE REQUIRED)

foreach(example_source ${EXAMPLE_SOURCES_3D})
    get_filename_component(example_target "${example_source}" NAME_WE)
    add_executable(${example_target} ${example_source})

    # Link with all the project modules
    target_link_libraries(${example_target} PRIVATE
        Geometry
        Topology
        Readers
        Meshing
        Export
        Common
    )

    target_link_libraries(${example_target} PRIVATE
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
    target_include_directories(${example_target} PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${OpenCASCADE_INCLUDE_DIR}
    )
endforeach()
