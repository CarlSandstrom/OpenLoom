# CMakeLists.txt for Meshing module
add_library(Meshing INTERFACE)

# Add subdirectories for meshing components
add_subdirectory(Data)
add_subdirectory(Operations)

# Link meshing submodules to the main Meshing library
target_link_libraries(Meshing INTERFACE MeshingData MeshingOperations)

target_include_directories(Meshing INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link with Geometry and Topology modules for meshing operations
target_link_libraries(Meshing INTERFACE Geometry Topology)
