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
