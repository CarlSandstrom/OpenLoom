# CMakeLists.txt for Geometry2D module (2D parametric geometry)
add_library(Geometry2D STATIC)

file(GLOB GEOMETRY2D_SOURCES CONFIGURE_DEPENDS "*.cpp")

target_sources(Geometry2D PRIVATE
    ${GEOMETRY2D_SOURCES}
)

target_include_directories(Geometry2D PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(Geometry2D PUBLIC
    Common
)
