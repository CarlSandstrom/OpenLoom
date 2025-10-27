# CMakeLists.txt for ReaderOpenCascade module
add_library(ReaderOpenCascade STATIC)

# Add source files when they exist
file(GLOB READEROPENCASCADE_SOURCES CONFIGURE_DEPENDS "*.cpp" "*.cc")
target_sources(ReaderOpenCascade PRIVATE 
    ${READEROPENCASCADE_SOURCES}
)

target_include_directories(ReaderOpenCascade INTERFACE 
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link with Geometry module for shape operations
target_link_libraries(ReaderOpenCascade PUBLIC Geometry Topology)