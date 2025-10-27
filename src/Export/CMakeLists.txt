# CMakeLists.txt for Export module
add_library(Export INTERFACE)

# Add source files when they exist
# file(GLOB EXPORT_SOURCES CONFIGURE_DEPENDS "*.cpp" "*.cc")
# target_sources(Export PRIVATE 
#     ${EXPORT_SOURCES}
# )

target_include_directories(Export INTERFACE 
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link with Geometry module for export operations
target_link_libraries(Export INTERFACE Geometry)
