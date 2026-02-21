# CMakeLists.txt for 2D Meshing Core
add_library(MeshingCore2D STATIC)

file(GLOB MESHING_CORE_2D_SOURCES CONFIGURE_DEPENDS "*.cpp")

target_sources(MeshingCore2D PRIVATE
    ${MESHING_CORE_2D_SOURCES}
)

target_include_directories(MeshingCore2D PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(MeshingCore2D PUBLIC
    MeshingData
    MeshingOperations
    Geometry
    Geometry2DBase
    Topology
    Topology2D
    Utils
    ExportCore
    spdlog::spdlog
    fmt::fmt
)

target_compile_definitions(MeshingCore2D PRIVATE SPDLOG_FMT_EXTERNAL)

if(OPENLOOM_USE_OPENMP)
    target_link_libraries(MeshingCore2D PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(MeshingCore2D PUBLIC OPENLOOM_HAS_OPENMP)
endif()
