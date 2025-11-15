# CMake configuration for Common module
add_library(Common INTERFACE)

target_include_directories(Common INTERFACE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if(TARGET Eigen3::Eigen)
    target_link_libraries(Common INTERFACE Eigen3::Eigen)
else()
    find_path(EIGEN3_INCLUDE_DIR Eigen/Core
        HINTS ${EIGEN3_INCLUDE_DIRS}
        PATH_SUFFIXES eigen3 Eigen3
    )

    if(EIGEN3_INCLUDE_DIR)
        target_include_directories(Common INTERFACE ${EIGEN3_INCLUDE_DIR})
    else()
        message(FATAL_ERROR "Eigen3 headers not found. Set Eigen3_DIR or install Eigen3.")
    endif()
endif()