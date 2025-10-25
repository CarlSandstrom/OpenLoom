# CMakeLists.txt for Export module
add_library(Export INTERFACE)

# Add source files when they exist
# target_sources(Export PRIVATE 
#     export.cpp
# )

target_include_directories(Export INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link with Geometry module for export operations
target_link_libraries(Export INTERFACE Geometry)
