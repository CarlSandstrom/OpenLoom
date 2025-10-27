# CMakeLists.txt for Meshing Operations module
add_library(MeshingOperations STATIC)

# Add all source files automatically
file(GLOB MESHING_OPERATIONS_SOURCES CONFIGURE_DEPENDS "*.cpp")
target_sources(MeshingOperations PRIVATE 
    ${MESHING_OPERATIONS_SOURCES}
)

target_include_directories(MeshingOperations PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link with MeshingData for access to mesh data structures
target_link_libraries(MeshingOperations PUBLIC 
    MeshingData
)