# Main CMakeLists.txt for src
add_subdirectory(Data)
add_subdirectory(Export)
add_subdirectory(Meshing)

add_executable(cMesh main.cc)

# Link libraries
find_package(VTK REQUIRED)

# Link all submodules (empty for now, will be populated later)
# target_link_libraries(cMesh PRIVATE Data Export Meshing)
find_package(Eigen3 REQUIRED)
find_package(spdlog REQUIRED)
find_package(fmt REQUIRED)

# Link external libraries
 target_link_libraries(cMesh PRIVATE 
    VTK::CommonCore
    Eigen3::Eigen
    spdlog::spdlog
    fmt::fmt
)
