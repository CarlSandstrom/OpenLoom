# CMakeLists.txt for 3D Meshing Core
add_library(MeshingCore3D STATIC)

file(GLOB MESHING_CORE_3D_SOURCES CONFIGURE_DEPENDS "*.cpp")

target_sources(MeshingCore3D PRIVATE
    ${MESHING_CORE_3D_SOURCES}
)

target_include_directories(MeshingCore3D PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(MeshingCore3D PUBLIC
    MeshingCore2D
    MeshingData
    MeshingOperations
    Geometry
    Geometry2DBase
    Topology
    Topology2D
    ExportCore
    spdlog::spdlog
    fmt::fmt
)

target_compile_definitions(MeshingCore3D PRIVATE SPDLOG_FMT_EXTERNAL)
