# CMakeLists.txt for Geometry OpenCascade module
add_library(GeometryOpenCascade STATIC
    OpenCascadeSurface.cpp
    OpenCascadeEdge.cpp
    OpenCascadeCorner.cpp
)

# Find OpenCASCADE
set(OpenCASCADE_DIR "$ENV{HOME}/lib/opencascade/lib/cmake/opencascade")
find_package(OpenCASCADE REQUIRED)

target_include_directories(GeometryOpenCascade PUBLIC 
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCASCADE_INCLUDE_DIR}
)

target_link_libraries(GeometryOpenCascade PUBLIC
    TKernel
    TKMath
    TKG3d
    TKG2d
    TKGeomBase
    TKBRep
    TKGeomAlgo
    TKTopAlgo
    TKDESTEP
    # TKSTEPBase
    # TKSTEPAttr
    # TKSTEP209
)