# CMakeLists.txt for Geometry OpenCascade module
add_library(GeometryOpenCascade INTERFACE)

# Add source files when they exist
# target_sources(GeometryOpenCascade PRIVATE 
#     opencascade_wrapper.cpp
# )

# Find OpenCASCADE
set(OpenCASCADE_DIR "$ENV{HOME}/lib/opencascade/lib/cmake/opencascade")
find_package(OpenCASCADE REQUIRED)

target_include_directories(GeometryOpenCascade INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OpenCASCADE_INCLUDE_DIR}
)

target_link_libraries(GeometryOpenCascade INTERFACE
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