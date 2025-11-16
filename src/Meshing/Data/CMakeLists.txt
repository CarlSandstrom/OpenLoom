# CMakeLists.txt for Meshing Data module
add_library(MeshingData STATIC)

# Add all source files automatically
file(GLOB MESHING_DATA_SOURCES CONFIGURE_DEPENDS "*.cpp")
target_sources(MeshingData PRIVATE 
    ${MESHING_DATA_SOURCES}
)

target_include_directories(MeshingData PUBLIC 
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(MeshingData PUBLIC
    Common
)