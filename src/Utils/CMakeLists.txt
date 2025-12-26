# CMakeLists.txt for Utils

add_library(Utils
    MeshLogger.cpp
)

target_link_libraries(Utils PUBLIC
    MeshingData
)

target_include_directories(Utils PUBLIC
    ${CMAKE_SOURCE_DIR}/src
)
