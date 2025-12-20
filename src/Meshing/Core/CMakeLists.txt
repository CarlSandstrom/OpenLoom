# CMakeLists.txt for Meshing Core orchestrator/strategies

# Add subdirectories for 2D and 3D meshing
add_subdirectory(2D)
add_subdirectory(3D)

# Create an interface library that aggregates both 2D and 3D
add_library(MeshingCore INTERFACE)

target_link_libraries(MeshingCore INTERFACE
    MeshingCore2D
    MeshingCore3D
)

target_include_directories(MeshingCore INTERFACE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)
