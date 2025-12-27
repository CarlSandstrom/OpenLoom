# CMakeLists.txt for Geometry2D OpenCascade module
add_library(Geometry2DOpenCascade STATIC
    OpenCascade2DEdge.cpp
    OpenCascade2DCorner.cpp
)

# Find OpenCASCADE
set(OpenCASCADE_DIR "$ENV{HOME}/lib/opencascade/lib/cmake/opencascade")
find_package(OpenCASCADE REQUIRED)

target_include_directories(Geometry2DOpenCascade PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCASCADE_INCLUDE_DIR}
)

target_link_libraries(Geometry2DOpenCascade PUBLIC
    Common
    Geometry2DBase
    TKernel
    TKMath
    TKG3d
    TKG2d
    TKGeomBase
    TKBRep
    TKGeomAlgo
    TKTopAlgo
)
