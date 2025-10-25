# CMakeLists.txt for ShapeConverter module
add_library(ShapeConverter INTERFACE)

# Add source files when they exist
# target_sources(ShapeConverter PRIVATE 
#     shape_converter.cpp
# )

target_include_directories(ShapeConverter INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link with Geometry module for shape operations
target_link_libraries(ShapeConverter INTERFACE Geometry)