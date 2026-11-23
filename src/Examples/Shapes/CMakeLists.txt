# Shape builders shared by the examples.

set(OpenCASCADE_DIR "$ENV{HOME}/lib/opencascade/lib/cmake/opencascade")
find_package(OpenCASCADE REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(ExampleShapes STATIC
    ExampleShapes2D.cpp
    ExampleShapes3D.cpp
)

target_link_libraries(ExampleShapes PUBLIC
    Geometry
    Topology2D
    Common
    Eigen3::Eigen
    TKernel
    TKMath
    TKG2d
    TKG3d
    TKBRep
    TKTopAlgo
    TKPrim
    TKBO
)

target_include_directories(ExampleShapes PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCASCADE_INCLUDE_DIR}
)
