# CMakeLists.txt for Meshing module
add_library(Meshing INTERFACE)

# Add source files when they exist
# target_sources(Meshing PRIVATE 
#     meshing.cpp
# )

target_include_directories(Meshing INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link with Geometry and Topology modules for meshing operations
target_link_libraries(Meshing INTERFACE Geometry Topology)
