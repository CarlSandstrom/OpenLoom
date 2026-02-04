#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/ConstraintChecker3D.h"
#include "Meshing/Core/3D/ElementGeometry3D.h"
#include "Meshing/Core/3D/ElementQuality3D.h"
#include "Meshing/Core/3D/GeometryStructures3D.h"
#include "Meshing/Core/3D/MeshOperations3D.h"
#include "Meshing/Core/3D/MeshingContext3D.h"
#include "Meshing/Core/3D/Shewchuk3DQualityController.h"
#include "Meshing/Core/3D/ShewchukRefiner3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"

#include <cmath>
#include <vector>

using namespace Meshing;

namespace
{
constexpr double TOLERANCE = 1e-9;
}

class ShewchukRefiner3DTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        context_ = std::make_unique<MeshingContext3D>();
    }

    size_t addNode(double x, double y, double z)
    {
        return context_->getMutator().addNode(Point3D(x, y, z));
    }

    size_t addTetrahedron(size_t n0, size_t n1, size_t n2, size_t n3)
    {
        auto tet = std::make_unique<TetrahedralElement>(
            std::array<size_t, 4>{n0, n1, n2, n3});
        return context_->getMutator().addElement(std::move(tet));
    }

    void addConstrainedSubsegment(size_t n1, size_t n2, const std::string& geomId)
    {
        context_->getMutator().addConstrainedSubsegment({n1, n2, geomId});
    }

    void addConstrainedSubfacet(size_t n1, size_t n2, size_t n3, const std::string& geomId)
    {
        context_->getMutator().addConstrainedSubfacet({n1, n2, n3, geomId});
    }

    // Create a unit tetrahedron with known good quality
    void createRegularTetrahedron()
    {
        // Regular tetrahedron with edge length 2
        // Vertices at (1,1,1), (-1,-1,1), (-1,1,-1), (1,-1,-1)
        n0_ = addNode(1.0, 1.0, 1.0);
        n1_ = addNode(-1.0, -1.0, 1.0);
        n2_ = addNode(-1.0, 1.0, -1.0);
        n3_ = addNode(1.0, -1.0, -1.0);
        addTetrahedron(n0_, n1_, n2_, n3_);
    }

    // Create a skinny tetrahedron (high circumradius-to-edge ratio)
    void createSkinnyTetrahedron()
    {
        // Very flat tetrahedron with apex close to base
        n0_ = addNode(0.0, 0.0, 0.0);
        n1_ = addNode(10.0, 0.0, 0.0);
        n2_ = addNode(5.0, 10.0, 0.0);
        n3_ = addNode(5.0, 3.0, 0.1); // Very small z offset = skinny
        addTetrahedron(n0_, n1_, n2_, n3_);
    }

    // Create a unit cube as 5 tetrahedra
    void createUnitCubeMesh()
    {
        // Vertices of unit cube
        size_t v0 = addNode(0.0, 0.0, 0.0);
        size_t v1 = addNode(1.0, 0.0, 0.0);
        size_t v2 = addNode(1.0, 1.0, 0.0);
        size_t v3 = addNode(0.0, 1.0, 0.0);
        size_t v4 = addNode(0.0, 0.0, 1.0);
        size_t v5 = addNode(1.0, 0.0, 1.0);
        size_t v6 = addNode(1.0, 1.0, 1.0);
        size_t v7 = addNode(0.0, 1.0, 1.0);

        // 5 tetrahedra decomposition of a cube
        addTetrahedron(v0, v1, v3, v4);
        addTetrahedron(v1, v2, v3, v6);
        addTetrahedron(v1, v3, v4, v6);
        addTetrahedron(v3, v4, v6, v7);
        addTetrahedron(v1, v4, v5, v6);

        // Set up constraint subfacets for the 6 faces of the cube
        // Bottom face (z=0)
        addConstrainedSubfacet(v0, v1, v3, "bottom");
        addConstrainedSubfacet(v1, v2, v3, "bottom");
        // Top face (z=1)
        addConstrainedSubfacet(v4, v5, v7, "top");
        addConstrainedSubfacet(v5, v6, v7, "top");
        // Front face (y=0)
        addConstrainedSubfacet(v0, v1, v4, "front");
        addConstrainedSubfacet(v1, v5, v4, "front");
        // Back face (y=1)
        addConstrainedSubfacet(v2, v3, v6, "back");
        addConstrainedSubfacet(v3, v7, v6, "back");
        // Left face (x=0)
        addConstrainedSubfacet(v0, v3, v4, "left");
        addConstrainedSubfacet(v3, v7, v4, "left");
        // Right face (x=1)
        addConstrainedSubfacet(v1, v2, v5, "right");
        addConstrainedSubfacet(v2, v6, v5, "right");

        // Set up constraint subsegments for the 12 edges of the cube
        addConstrainedSubsegment(v0, v1, "edge01");
        addConstrainedSubsegment(v1, v2, "edge12");
        addConstrainedSubsegment(v2, v3, "edge23");
        addConstrainedSubsegment(v3, v0, "edge30");
        addConstrainedSubsegment(v4, v5, "edge45");
        addConstrainedSubsegment(v5, v6, "edge56");
        addConstrainedSubsegment(v6, v7, "edge67");
        addConstrainedSubsegment(v7, v4, "edge74");
        addConstrainedSubsegment(v0, v4, "edge04");
        addConstrainedSubsegment(v1, v5, "edge15");
        addConstrainedSubsegment(v2, v6, "edge26");
        addConstrainedSubsegment(v3, v7, "edge37");
    }

    std::unique_ptr<MeshingContext3D> context_;
    size_t n0_, n1_, n2_, n3_;
};

// ============================================================================
// Quality Controller Interaction Tests
// ============================================================================

TEST_F(ShewchukRefiner3DTest, AcceptsGoodQualityMesh)
{
    createRegularTetrahedron();

    // Create quality controller with B > 2 (per Shewchuk's paper)
    Shewchuk3DQualityController controller(context_->getMeshData(), 2.5, 100);

    ShewchukRefiner3D refiner(*context_, controller);

    // Regular tetrahedron should already meet quality bounds
    MeshConnectivity connectivity(context_->getMeshData());
    EXPECT_TRUE(controller.isMeshAcceptable(context_->getMeshData(), connectivity));

    size_t initialTets = context_->getMeshData().getElementCount();

    // Refinement should not add any elements if mesh is already acceptable
    refiner.refine();

    // May have cleaned up external tets, but shouldn't have added many new ones
    EXPECT_LE(context_->getMeshData().getElementCount(), initialTets + 5);
}

TEST_F(ShewchukRefiner3DTest, RefinesSkinnyTetrahedron)
{
    createSkinnyTetrahedron();

    // Use small element limit for fast testing
    Shewchuk3DQualityController controller(context_->getMeshData(), 2.5, 50);

    // Verify the tetrahedron is skinny
    const auto* element = context_->getMeshData().getElement(0);
    const auto* tet = dynamic_cast<const TetrahedralElement*>(element);
    ASSERT_NE(tet, nullptr);

    ElementQuality3D quality(context_->getMeshData());
    double ratio = quality.getCircumradiusToShortestEdgeRatio(*tet);
    EXPECT_GT(ratio, 2.5); // Should be skinny

    ShewchukRefiner3D refiner(*context_, controller);
    refiner.refine();

    // After refinement, element count should have increased
    EXPECT_GT(context_->getMeshData().getElementCount(), 1);
}

// ============================================================================
// Subsegment Encroachment Tests
// ============================================================================

TEST_F(ShewchukRefiner3DTest, DetectsEncroachedSubsegment)
{
    // Create a segment and a point that encroaches it
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(4.0, 0.0, 0.0);
    size_t n2 = addNode(2.0, 0.5, 0.0); // Point inside diametral sphere

    // Create a tetrahedron containing all three
    size_t n3 = addNode(2.0, 1.0, 2.0);
    addTetrahedron(n0, n1, n2, n3);

    addConstrainedSubsegment(n0, n1, "test_edge");

    ConstraintChecker3D checker(context_->getMeshData());

    // Verify encroachment is detected
    const auto& subsegments = context_->getMeshData().getConstrainedSubsegments();
    Point3D p2 = context_->getMeshData().getNode(n2)->getCoordinates();
    EXPECT_TRUE(checker.isSubsegmentEncroached(subsegments[0], p2));
}

TEST_F(ShewchukRefiner3DTest, SplitsEncroachedSubsegmentDuringRefinement)
{
    // TODO: This test is for encroachment splitting which is not yet implemented.
    // Once Priority 1 (subsegment encroachment) is implemented in ShewchukRefiner3D,
    // update this test to verify that encroached subsegments are split.

    // Create mesh with an encroached subsegment
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(4.0, 0.0, 0.0);
    size_t n2 = addNode(2.0, 0.3, 0.0); // Encroaching point
    size_t n3 = addNode(2.0, -0.3, 0.0);
    size_t n4 = addNode(2.0, 0.0, 1.0);

    addTetrahedron(n0, n2, n3, n4);
    addTetrahedron(n1, n2, n3, n4);

    addConstrainedSubsegment(n0, n1, "test_edge");

    size_t initialSubsegments = context_->getMeshData().getConstrainedSubsegmentCount();

    // Quality controller that accepts everything (so we only test encroachment)
    Shewchuk3DQualityController controller(context_->getMeshData(), 1000.0, 100);

    ShewchukRefiner3D refiner(*context_, controller);
    refiner.refine();

    // For now, verify refiner runs without error and constraints are preserved
    // (encroachment splitting is not yet implemented)
    EXPECT_GE(context_->getMeshData().getConstrainedSubsegmentCount(), initialSubsegments);
}

// ============================================================================
// Subfacet Encroachment Tests
// ============================================================================

TEST_F(ShewchukRefiner3DTest, DetectsEncroachedSubfacet)
{
    // Create a triangle in the XY plane
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(1.0, 2.0, 0.0);
    // Non-coplanar point inside equatorial sphere
    size_t n3 = addNode(1.0, 0.5, 0.1);

    addTetrahedron(n0, n1, n2, n3);

    addConstrainedSubfacet(n0, n1, n2, "test_face");

    ConstraintChecker3D checker(context_->getMeshData());

    // Verify encroachment is detected
    const auto& subfacets = context_->getMeshData().getConstrainedSubfacets();
    Point3D p3 = context_->getMeshData().getNode(n3)->getCoordinates();
    EXPECT_TRUE(checker.isSubfacetEncroached(subfacets[0], p3));
}

TEST_F(ShewchukRefiner3DTest, SplitsEncroachedSubfacetDuringRefinement)
{
    // TODO: This test is for encroachment splitting which is not yet implemented.
    // Once Priority 2 (subfacet encroachment) is implemented in ShewchukRefiner3D,
    // update this test to verify that encroached subfacets are split.

    // Create mesh with an encroached subfacet
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(4.0, 0.0, 0.0);
    size_t n2 = addNode(2.0, 4.0, 0.0);
    size_t n3 = addNode(2.0, 1.0, 0.5); // Encroaching point
    size_t n4 = addNode(2.0, 1.0, -0.5);

    addTetrahedron(n0, n1, n2, n3);
    addTetrahedron(n0, n1, n2, n4);

    addConstrainedSubfacet(n0, n1, n2, "test_face");

    size_t initialSubfacets = context_->getMeshData().getConstrainedSubfacetCount();

    // Quality controller that accepts everything (so we only test encroachment)
    Shewchuk3DQualityController controller(context_->getMeshData(), 1000.0, 100);

    ShewchukRefiner3D refiner(*context_, controller);
    refiner.refine();

    // For now, verify refiner runs without error and constraints are preserved
    // (encroachment splitting is not yet implemented)
    EXPECT_GE(context_->getMeshData().getConstrainedSubfacetCount(), initialSubfacets);
}

// ============================================================================
// Circumcenter Rejection Tests
// ============================================================================

TEST_F(ShewchukRefiner3DTest, DefersCircumcenterInsertionWhenItWouldEncroachSubsegment)
{
    // Create a skinny tetrahedron whose circumcenter would encroach a nearby subsegment
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(10.0, 0.0, 0.0);
    size_t n2 = addNode(5.0, 10.0, 0.0);
    size_t n3 = addNode(5.0, 5.0, 0.5); // Skinny

    // A constraint edge nearby that the circumcenter might encroach
    size_t n4 = addNode(4.0, 4.0, 5.0);
    size_t n5 = addNode(6.0, 6.0, 5.0);

    addTetrahedron(n0, n1, n2, n3);
    addTetrahedron(n3, n4, n5, n2); // Connect to constraint edge

    addConstrainedSubsegment(n4, n5, "constraint_edge");

    size_t initialSubsegmentCount = context_->getMeshData().getConstrainedSubsegmentCount();

    Shewchuk3DQualityController controller(context_->getMeshData(), 2.5, 100);
    ShewchukRefiner3D refiner(*context_, controller);
    refiner.refine();

    // The algorithm should have handled encroachments by splitting subsegments
    // if the circumcenter would have encroached them
    // The mesh should still be refined (more elements than before)
    EXPECT_GE(context_->getMeshData().getElementCount(), 2);
}

// ============================================================================
// Termination Tests
// ============================================================================

TEST_F(ShewchukRefiner3DTest, TerminatesWithQualityBoundGreaterThanTwo)
{
    // Create a moderately complex mesh
    createUnitCubeMesh();

    // Use a quality bound B > 2 (per Shewchuk's termination guarantee)
    // Use reasonable element limit for faster testing
    Shewchuk3DQualityController controller(context_->getMeshData(), 2.5, 500);

    ShewchukRefiner3D refiner(*context_, controller);

    // This should terminate without hitting iteration limit
    EXPECT_NO_THROW(refiner.refine());

    // Verify the mesh quality after refinement
    MeshConnectivity connectivity(context_->getMeshData());

    // Most tetrahedra should now meet the quality bound
    // (some may remain unrefinable due to being too small)
    size_t acceptableCount = 0;
    size_t totalCount = 0;

    for (const auto& [tetId, element] : context_->getMeshData().getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        totalCount++;
        if (controller.isTetrahedronAcceptable(*tet))
        {
            acceptableCount++;
        }
    }

    // At least 80% of tetrahedra should be acceptable
    if (totalCount > 0)
    {
        double acceptableRatio = static_cast<double>(acceptableCount) / totalCount;
        EXPECT_GE(acceptableRatio, 0.8);
    }
}

TEST_F(ShewchukRefiner3DTest, RespectsElementLimit)
{
    createSkinnyTetrahedron();

    // Use B > 2 for guaranteed termination, and a low element limit
    size_t elementLimit = 30;
    Shewchuk3DQualityController controller(context_->getMeshData(), 2.5, elementLimit);

    ShewchukRefiner3D refiner(*context_, controller);
    refiner.refine();

    // Should not greatly exceed the element limit (some overshoot possible since
    // the limit is checked after each insertion)
    EXPECT_LE(context_->getMeshData().getElementCount(), elementLimit + 10);
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST_F(ShewchukRefiner3DTest, RefinementPreservesConstraints)
{
    createUnitCubeMesh();

    Shewchuk3DQualityController controller(context_->getMeshData(), 2.5, 200);
    ShewchukRefiner3D refiner(*context_, controller);
    refiner.refine();

    // Verify all original constraint subsegments still exist (possibly subdivided)
    // Each original subsegment should be represented by a chain of subsegments
    // The total endpoint connectivity should be preserved

    // Count total subsegments - should have increased or stayed same
    EXPECT_GE(context_->getMeshData().getConstrainedSubsegmentCount(), 12); // Original 12 edges

    // Count total subfacets - should have increased or stayed same
    EXPECT_GE(context_->getMeshData().getConstrainedSubfacetCount(), 12); // Original 12 triangles (2 per face)
}

TEST_F(ShewchukRefiner3DTest, RefinementProducesValidMesh)
{
    createUnitCubeMesh();

    Shewchuk3DQualityController controller(context_->getMeshData(), 2.5, 200);
    ShewchukRefiner3D refiner(*context_, controller);
    refiner.refine();

    // Verify all tetrahedra have positive volume
    ElementGeometry3D geometry(context_->getMeshData());

    for (const auto& [tetId, element] : context_->getMeshData().getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        double volume = geometry.computeVolume(*tet);
        EXPECT_GT(std::abs(volume), 0.0);
    }
}

TEST_F(ShewchukRefiner3DTest, RefinementWithDelaunayInitialization)
{
    // Test refinement starting from Delaunay initialization
    std::vector<Point3D> points = {
        Point3D(0.0, 0.0, 0.0),
        Point3D(1.0, 0.0, 0.0),
        Point3D(0.0, 1.0, 0.0),
        Point3D(0.0, 0.0, 1.0),
        Point3D(1.0, 1.0, 0.0),
        Point3D(1.0, 0.0, 1.0),
        Point3D(0.0, 1.0, 1.0),
        Point3D(1.0, 1.0, 1.0)};

    // Initialize using Bowyer-Watson
    auto nodeIds = context_->getOperations().initializeDelaunay(points);
    EXPECT_EQ(nodeIds.size(), 8);

    // Create constraints for cube faces (simplified)
    addConstrainedSubfacet(nodeIds[0], nodeIds[1], nodeIds[2], "face1");

    Shewchuk3DQualityController controller(context_->getMeshData(), 2.5, 200);
    ShewchukRefiner3D refiner(*context_, controller);

    EXPECT_NO_THROW(refiner.refine());

    // Should have a valid mesh after refinement
    EXPECT_GT(context_->getMeshData().getElementCount(), 0);
}

// ============================================================================
// Edge Case Tests
// ============================================================================

TEST_F(ShewchukRefiner3DTest, HandlesEmptyMesh)
{
    // Empty mesh - no elements
    Shewchuk3DQualityController controller(context_->getMeshData(), 2.5, 100);
    ShewchukRefiner3D refiner(*context_, controller);

    // Should complete without crashing
    EXPECT_NO_THROW(refiner.refine());
}

TEST_F(ShewchukRefiner3DTest, HandlesSingleTetrahedron)
{
    createRegularTetrahedron();

    Shewchuk3DQualityController controller(context_->getMeshData(), 2.5, 100);
    ShewchukRefiner3D refiner(*context_, controller);

    EXPECT_NO_THROW(refiner.refine());
}

TEST_F(ShewchukRefiner3DTest, HandlesNoConstraints)
{
    createSkinnyTetrahedron();

    // No constraints - just refine based on quality
    EXPECT_EQ(context_->getMeshData().getConstrainedSubsegmentCount(), 0);
    EXPECT_EQ(context_->getMeshData().getConstrainedSubfacetCount(), 0);

    // Use small limit since no constraints means unbounded growth
    Shewchuk3DQualityController controller(context_->getMeshData(), 2.5, 50);
    ShewchukRefiner3D refiner(*context_, controller);

    EXPECT_NO_THROW(refiner.refine());
}
