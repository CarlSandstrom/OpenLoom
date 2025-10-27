# CMakeLists.txt for tests
enable_testing()

# Collect all test source files
set(TEST_SOURCES
    test_main.cpp
    # Geometry tests
    Geometry/test_Corner.cpp
    Geometry/test_Edge.cpp
    Geometry/test_Surface.cpp
    
    # Meshing Data tests
    Meshing/Data/test_Node.cpp
    Meshing/Data/test_TetrahedralElement.cpp
    Meshing/Data/test_TriangularElement.cpp
    
    # Meshing Operations tests
    Meshing/Operations/test_MeshTransaction.cpp
    Meshing/Operations/test_MeshGeneration.cpp
    
    # Topology tests
    Topology/test_Corner.cpp
    Topology/test_Edge.cpp
    Topology/test_Surface.cpp
    
    # Integration tests
    Integration/test_MeshWorkflow.cpp
    Integration/test_GeometryToMesh.cpp
)

add_executable(runTests ${TEST_SOURCES})

# Link against all the libraries we want to test
target_link_libraries(runTests PRIVATE 
    # Google Test
    GTest::gtest 
    GTest::gtest_main 
    pthread
    
    # cMesh libraries
    Topology
    GeometryBase
    GeometryOpenCascade
    MeshingData
    MeshingOperations
    ReaderOpenCascade
    
    # Interface libraries (automatically bring in dependencies)
    Geometry
    Meshing
    Export
    Readers
    ShapeConverter
)

# Include directories for accessing headers
target_include_directories(runTests PRIVATE 
    ${CMAKE_SOURCE_DIR}/src
)

add_test(NAME AllTests COMMAND runTests)
