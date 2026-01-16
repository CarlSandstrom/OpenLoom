#include <gtest/gtest.h>
#include <memory>

#include "Common/Types.h"
#include "Meshing/Core/3D/ElementGeometry3D.h"
#include "Meshing/Core/3D/GeometryStructures3D.h"
#include "Meshing/Core/3D/MeshOperations3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"

using namespace Meshing;

namespace
{
constexpr double TOLERANCE = 1e-9;
}

class MeshOperations3DTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        mutator_ = std::make_unique<MeshMutator3D>(meshData_);
    }

    size_t addNode(double x, double y, double z)
    {
        return mutator_->addNode(Point3D(x, y, z));
    }

    size_t addTetrahedron(size_t n0, size_t n1, size_t n2, size_t n3)
    {
        auto tet = std::make_unique<TetrahedralElement>(
            std::array<size_t, 4>{n0, n1, n2, n3}
        );
        return mutator_->addElement(std::move(tet));
    }

    MeshData3D meshData_;
    std::unique_ptr<MeshMutator3D> mutator_;
};

// ============================================================================
// Find Conflicting Tetrahedra Tests
// ============================================================================

TEST_F(MeshOperations3DTest, FindConflictingTetrahedraFindsContainingTet)
{
    // Create a simple tetrahedron
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(1.0, 2.0, 0.0);
    size_t n3 = addNode(1.0, 0.5, 2.0);

    addTetrahedron(n0, n1, n2, n3);

    MeshOperations3D operations(meshData_);

    // Point inside the tetrahedron should be in its circumsphere
    Point3D centroid(1.0, 0.625, 0.5);
    auto conflicting = operations.findConflictingTetrahedra(centroid);

    EXPECT_EQ(conflicting.size(), 1);
}

TEST_F(MeshOperations3DTest, FindConflictingTetrahedraReturnsEmptyForFarPoint)
{
    // Create a simple tetrahedron at origin
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0, 0.0);
    size_t n2 = addNode(0.5, 1.0, 0.0);
    size_t n3 = addNode(0.5, 0.5, 1.0);

    addTetrahedron(n0, n1, n2, n3);

    MeshOperations3D operations(meshData_);

    // Point far from tetrahedron
    Point3D farPoint(100.0, 100.0, 100.0);
    auto conflicting = operations.findConflictingTetrahedra(farPoint);

    EXPECT_TRUE(conflicting.empty());
}

TEST_F(MeshOperations3DTest, FindConflictingTetrahedraMultipleTets)
{
    // Create two adjacent tetrahedra sharing a face
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(1.0, 2.0, 0.0);
    size_t n3 = addNode(1.0, 0.5, 2.0);
    size_t n4 = addNode(1.0, 0.5, -2.0);

    addTetrahedron(n0, n1, n2, n3);  // Above XY plane
    addTetrahedron(n0, n1, n2, n4);  // Below XY plane

    MeshOperations3D operations(meshData_);

    // Point at the shared face centroid - might conflict with both
    Point3D sharedFaceCentroid(1.0, 0.67, 0.0);
    auto conflicting = operations.findConflictingTetrahedra(sharedFaceCentroid);

    // Should find at least one tetrahedron
    EXPECT_GE(conflicting.size(), 1);
}

// ============================================================================
// Find Cavity Boundary Tests
// ============================================================================

TEST_F(MeshOperations3DTest, FindCavityBoundarySingleTet)
{
    // Create a single tetrahedron
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(1.0, 2.0, 0.0);
    size_t n3 = addNode(1.0, 0.5, 2.0);

    size_t tetId = addTetrahedron(n0, n1, n2, n3);

    MeshOperations3D operations(meshData_);

    std::vector<size_t> conflicting = {tetId};
    auto boundary = operations.findCavityBoundary(conflicting);

    // A single tetrahedron has 4 faces, all on boundary
    EXPECT_EQ(boundary.size(), 4);
}

TEST_F(MeshOperations3DTest, FindCavityBoundaryTwoAdjacentTets)
{
    // Create two tetrahedra sharing one face
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(1.0, 2.0, 0.0);
    size_t n3 = addNode(1.0, 0.5, 2.0);
    size_t n4 = addNode(1.0, 0.5, -2.0);

    size_t tetId1 = addTetrahedron(n0, n1, n2, n3);
    size_t tetId2 = addTetrahedron(n0, n1, n2, n4);

    MeshOperations3D operations(meshData_);

    std::vector<size_t> conflicting = {tetId1, tetId2};
    auto boundary = operations.findCavityBoundary(conflicting);

    // Two tets share one face, so boundary = 4 + 4 - 2 = 6 faces
    EXPECT_EQ(boundary.size(), 6);
}

// ============================================================================
// Bowyer-Watson Insertion Tests
// ============================================================================

TEST_F(MeshOperations3DTest, InsertVertexBowyerWatsonAddsNode)
{
    // Create MeshOperations first so it owns the mutator from the start
    // This avoids ID conflicts with the test fixture's mutator
    MeshOperations3D operations(meshData_);

    // Create initial tetrahedron using the operations' mutator
    size_t n0 = operations.getMutator().addNode(Point3D(0.0, 0.0, 0.0));
    size_t n1 = operations.getMutator().addNode(Point3D(2.0, 0.0, 0.0));
    size_t n2 = operations.getMutator().addNode(Point3D(1.0, 2.0, 0.0));
    size_t n3 = operations.getMutator().addNode(Point3D(1.0, 0.5, 2.0));

    auto tet = std::make_unique<TetrahedralElement>(
        std::array<size_t, 4>{n0, n1, n2, n3}
    );
    operations.getMutator().addElement(std::move(tet));

    size_t initialNodes = meshData_.getNodeCount();
    EXPECT_EQ(initialNodes, 4);

    // Insert point at centroid
    Point3D centroid(1.0, 0.625, 0.5);
    size_t newNodeId = operations.insertVertexBowyerWatson(centroid);

    // Should have one more node
    EXPECT_EQ(meshData_.getNodeCount(), initialNodes + 1);

    // Verify the new node exists and has correct coordinates
    const auto* newNode = meshData_.getNode(newNodeId);
    ASSERT_NE(newNode, nullptr);
    EXPECT_NEAR(newNode->getCoordinates().x(), 1.0, TOLERANCE);
    EXPECT_NEAR(newNode->getCoordinates().y(), 0.625, TOLERANCE);
    EXPECT_NEAR(newNode->getCoordinates().z(), 0.5, TOLERANCE);
}

TEST_F(MeshOperations3DTest, InsertVertexBowyerWatsonCreatesNewTetrahedra)
{
    // Create initial tetrahedron
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(1.0, 2.0, 0.0);
    size_t n3 = addNode(1.0, 0.5, 2.0);

    addTetrahedron(n0, n1, n2, n3);

    MeshOperations3D operations(meshData_);

    // Insert point at centroid
    Point3D centroid(1.0, 0.625, 0.5);
    operations.insertVertexBowyerWatson(centroid);

    // Should have 4 tetrahedra now (one per face of original tet)
    EXPECT_EQ(meshData_.getElementCount(), 4);
}

TEST_F(MeshOperations3DTest, InsertVertexBowyerWatsonOutsideExistingMesh)
{
    // Create initial tetrahedron
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0, 0.0);
    size_t n2 = addNode(0.5, 1.0, 0.0);
    size_t n3 = addNode(0.5, 0.5, 1.0);

    addTetrahedron(n0, n1, n2, n3);

    MeshOperations3D operations(meshData_);

    // Insert point far outside - should just add node
    Point3D farPoint(100.0, 100.0, 100.0);
    size_t newNodeId = operations.insertVertexBowyerWatson(farPoint);

    // Node should be added
    const auto* newNode = meshData_.getNode(newNodeId);
    ASSERT_NE(newNode, nullptr);
}

// ============================================================================
// Remove Tetrahedra Containing Node Tests
// ============================================================================

TEST_F(MeshOperations3DTest, RemoveTetrahedraContainingNodeRemovesConnectedTets)
{
    // Create two tetrahedra sharing a node
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(1.0, 2.0, 0.0);
    size_t n3 = addNode(1.0, 0.5, 2.0);
    size_t n4 = addNode(3.0, 0.0, 0.0);

    addTetrahedron(n0, n1, n2, n3);
    addTetrahedron(n1, n2, n3, n4);

    EXPECT_EQ(meshData_.getElementCount(), 2);

    MeshOperations3D operations(meshData_);

    // Remove tets containing n1 (both tets)
    bool removed = operations.removeTetrahedraContainingNode(n1);

    EXPECT_TRUE(removed);
    EXPECT_EQ(meshData_.getElementCount(), 0);
}

TEST_F(MeshOperations3DTest, RemoveTetrahedraContainingNodeReturnsFalseForUnusedNode)
{
    // Create a tetrahedron
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0, 0.0);
    size_t n2 = addNode(0.5, 1.0, 0.0);
    size_t n3 = addNode(0.5, 0.5, 1.0);

    // Add an isolated node
    size_t isolated = addNode(10.0, 10.0, 10.0);

    addTetrahedron(n0, n1, n2, n3);

    MeshOperations3D operations(meshData_);

    // Remove tets containing isolated node - should remove nothing
    bool removed = operations.removeTetrahedraContainingNode(isolated);

    EXPECT_FALSE(removed);
    EXPECT_EQ(meshData_.getElementCount(), 1);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(MeshOperations3DTest, FindConflictingTetrahedraEmptyMesh)
{
    MeshOperations3D operations(meshData_);

    Point3D anyPoint(1.0, 1.0, 1.0);
    auto conflicting = operations.findConflictingTetrahedra(anyPoint);

    EXPECT_TRUE(conflicting.empty());
}

TEST_F(MeshOperations3DTest, FindCavityBoundaryEmptyList)
{
    MeshOperations3D operations(meshData_);

    std::vector<size_t> empty;
    auto boundary = operations.findCavityBoundary(empty);

    EXPECT_TRUE(boundary.empty());
}

TEST_F(MeshOperations3DTest, InsertVertexBowyerWatsonEmptyMesh)
{
    MeshOperations3D operations(meshData_);

    // Insert into empty mesh - should just add node
    Point3D point(1.0, 1.0, 1.0);
    size_t nodeId = operations.insertVertexBowyerWatson(point);

    EXPECT_EQ(meshData_.getNodeCount(), 1);

    const auto* node = meshData_.getNode(nodeId);
    ASSERT_NE(node, nullptr);
    EXPECT_NEAR(node->getCoordinates().x(), 1.0, TOLERANCE);
}

// ============================================================================
// Create Bounding Tetrahedron Tests
// ============================================================================

TEST_F(MeshOperations3DTest, CreateBoundingTetrahedronCreatesValidTet)
{
    MeshOperations3D operations(meshData_);

    std::vector<Point3D> points = {
        Point3D(0.0, 0.0, 0.0),
        Point3D(1.0, 0.0, 0.0),
        Point3D(0.0, 1.0, 0.0),
        Point3D(0.0, 0.0, 1.0)
    };

    auto boundingIds = operations.createBoundingTetrahedron(points);

    // Should have 4 nodes and 1 element
    EXPECT_EQ(meshData_.getNodeCount(), 4);
    EXPECT_EQ(meshData_.getElementCount(), 1);

    // All 4 bounding node IDs should be valid
    for (size_t id : boundingIds)
    {
        EXPECT_NE(meshData_.getNode(id), nullptr);
    }
}

TEST_F(MeshOperations3DTest, CreateBoundingTetrahedronContainsAllPoints)
{
    MeshOperations3D operations(meshData_);

    std::vector<Point3D> points = {
        Point3D(1.0, 2.0, 3.0),
        Point3D(4.0, 5.0, 6.0),
        Point3D(-1.0, -2.0, -3.0),
        Point3D(2.0, 1.0, 0.5)
    };

    auto boundingIds = operations.createBoundingTetrahedron(points);

    // Get the bounding tetrahedron vertices
    Point3D v0 = meshData_.getNode(boundingIds[0])->getCoordinates();
    Point3D v1 = meshData_.getNode(boundingIds[1])->getCoordinates();
    Point3D v2 = meshData_.getNode(boundingIds[2])->getCoordinates();
    Point3D v3 = meshData_.getNode(boundingIds[3])->getCoordinates();

    // Verify bounding box of bounding tet contains all input points
    double minX = std::min({v0.x(), v1.x(), v2.x(), v3.x()});
    double maxX = std::max({v0.x(), v1.x(), v2.x(), v3.x()});
    double minY = std::min({v0.y(), v1.y(), v2.y(), v3.y()});
    double maxY = std::max({v0.y(), v1.y(), v2.y(), v3.y()});
    double minZ = std::min({v0.z(), v1.z(), v2.z(), v3.z()});
    double maxZ = std::max({v0.z(), v1.z(), v2.z(), v3.z()});

    for (const auto& p : points)
    {
        EXPECT_LT(p.x(), maxX);
        EXPECT_GT(p.x(), minX);
        EXPECT_LT(p.y(), maxY);
        EXPECT_GT(p.y(), minY);
        EXPECT_LT(p.z(), maxZ);
        EXPECT_GT(p.z(), minZ);
    }
}

TEST_F(MeshOperations3DTest, CreateBoundingTetrahedronHandlesEmptyInput)
{
    MeshOperations3D operations(meshData_);

    std::vector<Point3D> emptyPoints;
    auto boundingIds = operations.createBoundingTetrahedron(emptyPoints);

    // Should return dummy IDs (all zeros) for empty input
    EXPECT_EQ(boundingIds[0], 0);
    EXPECT_EQ(boundingIds[1], 0);
    EXPECT_EQ(boundingIds[2], 0);
    EXPECT_EQ(boundingIds[3], 0);
}

TEST_F(MeshOperations3DTest, CreateBoundingTetrahedronHandlesSinglePoint)
{
    MeshOperations3D operations(meshData_);

    std::vector<Point3D> points = {Point3D(5.0, 5.0, 5.0)};

    auto boundingIds = operations.createBoundingTetrahedron(points);

    // Should still create valid bounding tetrahedron
    EXPECT_EQ(meshData_.getNodeCount(), 4);
    EXPECT_EQ(meshData_.getElementCount(), 1);
}

// ============================================================================
// Initialize Delaunay Tests
// ============================================================================

TEST_F(MeshOperations3DTest, InitializeDelaunayInsertsAllPoints)
{
    MeshOperations3D operations(meshData_);

    std::vector<Point3D> points = {
        Point3D(0.5, 0.5, 0.5),
        Point3D(1.5, 0.5, 0.5),
        Point3D(1.0, 1.5, 0.5),
        Point3D(1.0, 1.0, 1.5)
    };

    auto nodeIds = operations.initializeDelaunay(points);

    // Should return same number of node IDs as input points
    EXPECT_EQ(nodeIds.size(), points.size());

    // Each returned node ID should be valid
    for (size_t id : nodeIds)
    {
        EXPECT_NE(meshData_.getNode(id), nullptr);
    }

    // Total nodes = 4 bounding + 4 input = 8
    EXPECT_EQ(meshData_.getNodeCount(), 8);
}

TEST_F(MeshOperations3DTest, InitializeDelaunayCreatesTetrahedra)
{
    MeshOperations3D operations(meshData_);

    std::vector<Point3D> points = {
        Point3D(0.5, 0.5, 0.5),
        Point3D(1.5, 0.5, 0.5),
        Point3D(1.0, 1.5, 0.5),
        Point3D(1.0, 1.0, 1.5)
    };

    operations.initializeDelaunay(points);

    // Should have some tetrahedra (exact count depends on geometry)
    EXPECT_GT(meshData_.getElementCount(), 0);
}

TEST_F(MeshOperations3DTest, InitializeDelaunayReturnsEmptyForEmptyInput)
{
    MeshOperations3D operations(meshData_);

    std::vector<Point3D> emptyPoints;
    auto nodeIds = operations.initializeDelaunay(emptyPoints);

    EXPECT_TRUE(nodeIds.empty());
    EXPECT_EQ(meshData_.getNodeCount(), 0);
    EXPECT_EQ(meshData_.getElementCount(), 0);
}

TEST_F(MeshOperations3DTest, InitializeDelaunayPreservesPointOrder)
{
    MeshOperations3D operations(meshData_);

    std::vector<Point3D> points = {
        Point3D(1.0, 2.0, 3.0),
        Point3D(4.0, 5.0, 6.0),
        Point3D(7.0, 8.0, 9.0)
    };

    auto nodeIds = operations.initializeDelaunay(points);

    // Verify each node has coordinates matching input order
    for (size_t i = 0; i < points.size(); ++i)
    {
        const auto* node = meshData_.getNode(nodeIds[i]);
        ASSERT_NE(node, nullptr);
        EXPECT_NEAR(node->getCoordinates().x(), points[i].x(), TOLERANCE);
        EXPECT_NEAR(node->getCoordinates().y(), points[i].y(), TOLERANCE);
        EXPECT_NEAR(node->getCoordinates().z(), points[i].z(), TOLERANCE);
    }
}

// ============================================================================
// Remove Bounding Tetrahedron Tests
// ============================================================================

TEST_F(MeshOperations3DTest, RemoveBoundingTetrahedronRemovesBoundingNodes)
{
    MeshOperations3D operations(meshData_);

    std::vector<Point3D> points = {
        Point3D(0.5, 0.5, 0.5),
        Point3D(1.5, 0.5, 0.5),
        Point3D(1.0, 1.5, 0.5),
        Point3D(1.0, 1.0, 1.5)
    };

    // Create bounding tetrahedron and insert points
    auto boundingIds = operations.createBoundingTetrahedron(points);
    for (const auto& p : points)
    {
        operations.insertVertexBowyerWatson(p);
    }

    size_t nodesBeforeRemoval = meshData_.getNodeCount();

    // Remove bounding tetrahedron
    operations.removeBoundingTetrahedron(boundingIds);

    // Should have 4 fewer nodes (the bounding vertices)
    EXPECT_EQ(meshData_.getNodeCount(), nodesBeforeRemoval - 4);

    // Bounding nodes should no longer exist
    for (size_t id : boundingIds)
    {
        EXPECT_EQ(meshData_.getNode(id), nullptr);
    }
}

TEST_F(MeshOperations3DTest, RemoveBoundingTetrahedronRemovesConnectedTetrahedra)
{
    MeshOperations3D operations(meshData_);

    std::vector<Point3D> points = {
        Point3D(0.5, 0.5, 0.5),
        Point3D(1.5, 0.5, 0.5),
        Point3D(1.0, 1.5, 0.5),
        Point3D(1.0, 1.0, 1.5)
    };

    auto boundingIds = operations.createBoundingTetrahedron(points);
    for (const auto& p : points)
    {
        operations.insertVertexBowyerWatson(p);
    }

    size_t elementsBeforeRemoval = meshData_.getElementCount();
    EXPECT_GT(elementsBeforeRemoval, 0);

    operations.removeBoundingTetrahedron(boundingIds);

    // Should have fewer elements (all elements touching bounding nodes removed)
    EXPECT_LT(meshData_.getElementCount(), elementsBeforeRemoval);
}

TEST_F(MeshOperations3DTest, FullDelaunayWorkflow)
{
    MeshOperations3D operations(meshData_);

    // Create a simple set of points forming a cube
    std::vector<Point3D> points = {
        Point3D(0.0, 0.0, 0.0),
        Point3D(1.0, 0.0, 0.0),
        Point3D(0.0, 1.0, 0.0),
        Point3D(1.0, 1.0, 0.0),
        Point3D(0.0, 0.0, 1.0),
        Point3D(1.0, 0.0, 1.0),
        Point3D(0.0, 1.0, 1.0),
        Point3D(1.0, 1.0, 1.0)
    };

    // Create bounding tetrahedron
    auto boundingIds = operations.createBoundingTetrahedron(points);
    EXPECT_EQ(meshData_.getNodeCount(), 4);
    EXPECT_EQ(meshData_.getElementCount(), 1);

    // Insert all points
    std::vector<size_t> nodeIds;
    for (const auto& p : points)
    {
        nodeIds.push_back(operations.insertVertexBowyerWatson(p));
    }
    EXPECT_EQ(nodeIds.size(), 8);
    EXPECT_EQ(meshData_.getNodeCount(), 12); // 4 bounding + 8 input

    // Remove bounding tetrahedron
    operations.removeBoundingTetrahedron(boundingIds);
    EXPECT_EQ(meshData_.getNodeCount(), 8); // Only input points remain

    // Verify all input points still exist
    for (size_t id : nodeIds)
    {
        EXPECT_NE(meshData_.getNode(id), nullptr);
    }

    // Should have tetrahedra remaining (forming Delaunay triangulation of cube)
    EXPECT_GT(meshData_.getElementCount(), 0);
}
