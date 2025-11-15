# CMakeLists.txt for Topology module
add_library(Topology STATIC)

# Add all source files automatically
file(GLOB TOPOLOGY_SOURCES CONFIGURE_DEPENDS "*.cpp" "*.cc")
target_sources(Topology PRIVATE 
    ${TOPOLOGY_SOURCES}
)

target_include_directories(Topology PUBLIC 
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link with Geometry module for topology operations
target_link_libraries(Topology PUBLIC Geometry Common)