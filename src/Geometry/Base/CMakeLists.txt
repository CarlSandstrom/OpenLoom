# CMakeLists.txt for Geometry Base module
add_library(GeometryBase INTERFACE)

# Add source files when they exist
# target_sources(GeometryBase PRIVATE 
#     geometry_base.cpp
# )

target_include_directories(GeometryBase INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)