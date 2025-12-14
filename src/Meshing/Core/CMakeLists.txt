# CMakeLists.txt for Meshing Core orchestrator/strategies
add_library(MeshingCore STATIC)

file(GLOB MESHING_CORE_SOURCES CONFIGURE_DEPENDS "*.cpp")

target_sources(MeshingCore PRIVATE
    ${MESHING_CORE_SOURCES}
)

target_include_directories(MeshingCore PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link to meshing data/operations and topology/geometry
# Use PUBLIC so consumers of MeshingCore also see these deps
# Geometry and Topology are interface aggregates

# MeshingCore itself uses only headers from Geometry/Topology for now,
# but we link them to be explicit for future calls from .cpp files

target_link_libraries(MeshingCore PUBLIC
    MeshingData
    MeshingOperations
    Geometry
    Geometry2D
    Topology
    Topology2D
    ExportCore
    spdlog::spdlog
    fmt::fmt
)

target_compile_definitions(MeshingCore PRIVATE SPDLOG_FMT_EXTERNAL)
