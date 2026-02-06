# CMakeLists.txt for tests
enable_testing()

# Collect all test source files
set(TEST_SOURCES
    test_main.cpp

    # Common tests
    Common/Exceptions/test_Exception.cpp

    # Geometry tests
    Geometry/test_Corner.cpp
    Geometry/test_Edge.cpp
    Geometry/test_Surface.cpp
    
    # Meshing Data tests
    Meshing/Data/test_Node.cpp
    Meshing/Data/test_TetrahedralElement.cpp
    
    # Meshing Core tests
    Meshing/Core/test_ConstrainedDelaunay2D.cpp
    Meshing/Core/test_GeometryUtilities2D.cpp
    Meshing/Core/test_ElementGeometry2D.cpp
    Meshing/Core/test_ElementQuality2D.cpp
    Meshing/Core/test_ConstraintChecker2D.cpp
    Meshing/Core/test_GeometryUtilities3D.cpp
    Meshing/Core/test_ElementGeometry3D.cpp
    Meshing/Core/test_ElementQuality3D.cpp
    Meshing/Core/test_ConstraintChecker3D.cpp
    Meshing/Core/test_MeshOperations3D.cpp
    Meshing/Core/test_Shewchuk2DQualityController.cpp
    Meshing/Core/test_ShewchukRefiner3D.cpp
    Meshing/Core/test_FacetTriangulation.cpp
    Meshing/Core/test_FacetTriangulationManager.cpp
    Meshing/Core/test_ConstrainedDelaunay3D.cpp

    # Meshing Operations tests
    Meshing/Operations/test_MeshTransaction.cpp
    
    # Topology tests
    Topology/test_Corner.cpp
    Topology/test_Edge.cpp
    Topology/test_Surface.cpp
    
    # Integration tests
    Integration/test_MeshWorkflow.cpp
    Integration/test_GeometryToMesh.cpp
    Integration/test_VtkExporter.cpp
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
    MeshingData
    MeshingOperations

    # Interface libraries (automatically bring in dependencies)
    Geometry
    Meshing
    Export
    Readers
    ShapeConverter
    Common
)

# Include directories for accessing headers
target_include_directories(runTests PRIVATE 
    ${CMAKE_SOURCE_DIR}/src
)

add_test(NAME AllTests COMMAND runTests)
