# CMakeLists.txt for ShapeConverter module
add_library(ShapeConverter INTERFACE)

# Add source files when they exist
# file(GLOB SHAPECONVERTER_SOURCES CONFIGURE_DEPENDS "*.cpp" "*.cc")
# target_sources(ShapeConverter PRIVATE 
#     ${SHAPECONVERTER_SOURCES}
# )

target_include_directories(ShapeConverter INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link with Geometry module for shape operations
target_link_libraries(ShapeConverter INTERFACE Geometry)