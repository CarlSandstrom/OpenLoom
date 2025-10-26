# CMakeLists.txt for Readers module
add_library(Readers INTERFACE)

# Add subdirectories
add_subdirectory(ShapeConverter)
add_subdirectory(OpenCascade)

# Link readers submodules to the main Readers library
target_link_libraries(Readers INTERFACE ShapeConverter ReaderOpenCascade)

target_include_directories(Readers INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
)