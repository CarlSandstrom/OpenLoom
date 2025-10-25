# CMakeLists.txt for Topology module
add_library(Topology INTERFACE)

# Add source files when they exist
target_sources(Topology PRIVATE 
     TopoDS_ShapeConverter.h
)

target_include_directories(Topology INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link with Geometry module for topology operations
target_link_libraries(Topology INTERFACE Geometry)