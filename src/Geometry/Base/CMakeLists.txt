# CMakeLists.txt for Geometry Base module
add_library(GeometryBase STATIC)

# Add all source files automatically
file(GLOB GEOMETRY_BASE_SOURCES CONFIGURE_DEPENDS "*.cpp")
target_sources(GeometryBase PRIVATE 
    ${GEOMETRY_BASE_SOURCES}
)

target_include_directories(GeometryBase PUBLIC 
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)