# CMakeLists.txt for Geometry module
add_library(Geometry INTERFACE)

# Add subdirectories for different geometry implementations
add_subdirectory(Base)
add_subdirectory(OpenCascade)

# Link geometry submodules to the main Geometry library
target_link_libraries(Geometry INTERFACE GeometryBase GeometryOpenCascade)

target_include_directories(Geometry INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)