add_library(Exceptions INTERFACE)

target_sources(Exceptions
    INTERFACE
        Exception.h
        GeometryException.h
        MeshException.h
        TopologyException.h
)

target_include_directories(Exceptions INTERFACE
    ${CMAKE_SOURCE_DIR}/src
)
