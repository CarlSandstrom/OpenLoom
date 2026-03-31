# CMakeLists.txt for Meshing Data module
add_library(MeshingData STATIC)

# Add all source files from subdirectories
file(GLOB MESHING_DATA_SOURCES CONFIGURE_DEPENDS "*.cpp")
file(GLOB MESHING_DATA_BASE_SOURCES CONFIGURE_DEPENDS "Base/*.cpp")
file(GLOB MESHING_DATA_2D_SOURCES CONFIGURE_DEPENDS "2D/*.cpp")
file(GLOB MESHING_DATA_3D_SOURCES CONFIGURE_DEPENDS "3D/*.cpp")

target_sources(MeshingData PRIVATE
    ${MESHING_DATA_SOURCES}
    ${MESHING_DATA_BASE_SOURCES}
    ${MESHING_DATA_2D_SOURCES}
    ${MESHING_DATA_3D_SOURCES}
)

target_include_directories(MeshingData PUBLIC 
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(MeshingData PUBLIC
    Common
)