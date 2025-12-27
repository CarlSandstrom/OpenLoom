# CMakeLists.txt for Geometry module
add_library(Geometry INTERFACE)

# Add subdirectories for different geometry implementations
add_subdirectory(2D/Base)
# add_subdirectory(2D/OpenCascade)  # Uncomment when 2D OpenCascade implementations are added
add_subdirectory(3D/Base)
add_subdirectory(3D/OpenCascade)

# Link geometry submodules to the main Geometry library
target_link_libraries(Geometry INTERFACE Geometry2DBase GeometryBase GeometryOpenCascade)

target_include_directories(Geometry INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)