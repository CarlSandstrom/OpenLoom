# CMakeLists.txt for Geometry2D Base module
add_library(Geometry2DBase STATIC)

# Add all source files automatically
file(GLOB GEOMETRY2D_BASE_SOURCES CONFIGURE_DEPENDS "*.cpp")
target_sources(Geometry2DBase PRIVATE
    ${GEOMETRY2D_BASE_SOURCES}
)

target_include_directories(Geometry2DBase PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(Geometry2DBase PUBLIC
    Common
    GeometryBase
)
