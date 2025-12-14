# CMakeLists.txt for Topology2D module (2D parametric topology)
add_library(Topology2D STATIC)

file(GLOB TOPOLOGY2D_SOURCES CONFIGURE_DEPENDS "*.cpp")

target_sources(Topology2D PRIVATE
    ${TOPOLOGY2D_SOURCES}
)

target_include_directories(Topology2D PUBLIC
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)
