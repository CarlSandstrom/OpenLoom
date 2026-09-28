#include <algorithm>
#include <cmath>
#include <gtest/gtest.h>
#include <memory>
#include <numbers>

#include "Common/Exceptions/MeshException.h"
#include "Common/Types.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/CurveSegmentManager.h"

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
            std::array<size_t, 4>{n0, n1, n2, n3});
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
    auto conflicting = operations.getQueries().findConflictingTetrahedra(centroid);

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
    auto conflicting = operations.getQueries().findConflictingTetrahedra(farPoint);

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

    addTetrahedron(n0, n1, n2, n3); // Above XY plane
    addTetrahedron(n0, n1, n2, n4); // Below XY plane

    MeshOperations3D operations(meshData_);

    // Point at the shared face centroid - might conflict with both
    Point3D sharedFaceCentroid(1.0, 0.67, 0.0);
    auto conflicting = operations.getQueries().findConflictingTetrahedra(sharedFaceCentroid);

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
    auto boundary = operations.getQueries().findCavityBoundary(conflicting);

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
    auto boundary = operations.getQueries().findCavityBoundary(conflicting);

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
        std::array<size_t, 4>{n0, n1, n2, n3});
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

TEST_F(MeshOperations3DTest, InsertVertexBowyerWatsonExtendsCavityThroughCoplanarFace)
{
    // Reproduces OPE-149: a coarse cylinder-like point cloud (radius 3, height 8,
    // 3 points per circle -- matching RCDTMesherCylinderTest) where the midpoint of
    // a top-circle arc lands exactly in the plane of an existing cap face. Before
    // the fix, MeshOperations3D::retriangulate() silently skipped fanning a
    // (degenerate) tetrahedron onto that coplanar face, leaving it uncovered --
    // a real gap in the tetrahedralization (confirmed during investigation via a
    // point-in-tetrahedron volume check, not just a floating-point near-miss).
    // A later fix (OPE-173) replaced the original cavity-flood-fill workaround
    // with Simulation of Simplicity in RobustPredicates3D, which resolves this
    // coplanarity (and the in-sphere conflict test around it) directly, so the
    // new vertex is never left orphaned.
    //
    // The bounding tetrahedron is intentionally kept in the mesh (mirroring how
    // RCDTMesher now defers its removal until after refinement).
    constexpr double RADIUS = 3.0;
    constexpr double HEIGHT = 8.0;

    std::vector<Point3D> points;
    for (int i = 0; i < 3; ++i)
    {
        const double angle = i * 2.0 * std::numbers::pi / 3.0;
        points.emplace_back(RADIUS * std::cos(angle), RADIUS * std::sin(angle), HEIGHT);
        points.emplace_back(RADIUS * std::cos(angle), RADIUS * std::sin(angle), 0.0);
    }

    MeshOperations3D operations(meshData_);
    operations.createBoundingTetrahedron(points);
    for (const auto& p : points)
    {
        operations.insertVertexBowyerWatson(p);
    }

    // Midpoint of the arc between the first two top-circle points -- lies in the
    // same plane (z = HEIGHT) as the existing top-cap faces.
    const double midAngle = std::numbers::pi / 3.0;
    const Point3D midpoint(RADIUS * std::cos(midAngle), RADIUS * std::sin(midAngle), HEIGHT);

    size_t midNodeId = 0;
    EXPECT_NO_THROW(midNodeId = operations.insertVertexBowyerWatson(midpoint));

    bool participatesInATet = false;
    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;
        const auto& ids = tet->getNodeIds();
        if (std::find(ids.begin(), ids.end(), midNodeId) != ids.end())
        {
            participatesInATet = true;
            break;
        }
    }
    EXPECT_TRUE(participatesInATet) << "Inserted vertex is not part of any tetrahedron -- "
                                        "the coplanar cavity boundary face was left uncovered";
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(MeshOperations3DTest, FindConflictingTetrahedraEmptyMesh)
{
    MeshOperations3D operations(meshData_);

    Point3D anyPoint(1.0, 1.0, 1.0);
    auto conflicting = operations.getQueries().findConflictingTetrahedra(anyPoint);

    EXPECT_TRUE(conflicting.empty());
}

TEST_F(MeshOperations3DTest, FindCavityBoundaryEmptyList)
{
    MeshOperations3D operations(meshData_);

    std::vector<size_t> empty;
    auto boundary = operations.getQueries().findCavityBoundary(empty);

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
        Point3D(0.0, 0.0, 1.0)};

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
        Point3D(2.0, 1.0, 0.5)};

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

    // Should throw on empty input (programming error)
    EXPECT_THROW(operations.createBoundingTetrahedron(emptyPoints), OpenLoom::MeshException);
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
// Face Adjacency Tests
// ============================================================================

TEST_F(MeshOperations3DTest, FindTetrahedraWithFaceReturnsCorrectTets)
{
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(1.0, 2.0, 0.0);
    size_t n3 = addNode(1.0, 0.5, 2.0);
    size_t n4 = addNode(1.0, 0.5, -2.0);

    addTetrahedron(n0, n1, n2, n3); // Tet above
    addTetrahedron(n0, n1, n2, n4); // Tet below

    MeshOperations3D operations(meshData_);

    // Face n0-n1-n2 is shared by both tetrahedra
    auto tets = operations.getQueries().findTetrahedraWithFace(n0, n1, n2);
    EXPECT_EQ(tets.size(), 2);

    // Face n0-n1-n3 is only in the first tetrahedron
    tets = operations.getQueries().findTetrahedraWithFace(n0, n1, n3);
    EXPECT_EQ(tets.size(), 1);

    // Face n0-n1-n4 is only in the second tetrahedron
    tets = operations.getQueries().findTetrahedraWithFace(n0, n1, n4);
    EXPECT_EQ(tets.size(), 1);
}

TEST_F(MeshOperations3DTest, FindOppositeVertexReturnsCorrectNode)
{
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0, 0.0);
    size_t n2 = addNode(0.5, 1.0, 0.0);
    size_t n3 = addNode(0.5, 0.5, 1.0);
    size_t tetId = addTetrahedron(n0, n1, n2, n3);

    MeshOperations3D operations(meshData_);

    // n3 is opposite to face n0-n1-n2
    EXPECT_EQ(operations.getQueries().findOppositeVertex(tetId, n0, n1, n2), n3);

    // n0 is opposite to face n1-n2-n3
    EXPECT_EQ(operations.getQueries().findOppositeVertex(tetId, n1, n2, n3), n0);

    // n1 is opposite to face n0-n2-n3
    EXPECT_EQ(operations.getQueries().findOppositeVertex(tetId, n0, n2, n3), n1);

    // n2 is opposite to face n0-n1-n3
    EXPECT_EQ(operations.getQueries().findOppositeVertex(tetId, n0, n1, n3), n2);
}

TEST_F(MeshOperations3DTest, FindOppositeVertexInvalidTetReturnsSizeMax)
{
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0, 0.0);
    size_t n2 = addNode(0.5, 1.0, 0.0);
    size_t n3 = addNode(0.5, 0.5, 1.0);
    addTetrahedron(n0, n1, n2, n3);

    MeshOperations3D operations(meshData_);

    // Invalid tetrahedron ID
    EXPECT_EQ(operations.getQueries().findOppositeVertex(9999, n0, n1, n2), SIZE_MAX);
}
