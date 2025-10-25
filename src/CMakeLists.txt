# Main CMakeLists.txt for src
add_subdirectory(Geometry)
add_subdirectory(Topology)
add_subdirectory(Readers)
add_subdirectory(Export)
add_subdirectory(Meshing)
add_subdirectory(Examples)

# Note: Individual executables are now built in their respective subdirectories
# The Examples subdirectory contains the main executable (cMeshExample)
