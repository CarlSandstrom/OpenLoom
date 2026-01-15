#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/ConstraintChecker3D.h"
#include "Meshing/Core/3D/GeometryStructures3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"

using namespace Meshing;

class ConstraintChecker3DTest : public ::testing::Test
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

    MeshData3D meshData_;
    std::unique_ptr<MeshMutator3D> mutator_;
};

// ============================================================================
// Subsegment Encroachment Tests (Diametral Sphere)
// ============================================================================

TEST_F(ConstraintChecker3DTest, IsSubsegmentEncroachedDetectsPointInDiametralSphere)
{
    // Create a segment along the X axis from (0,0,0) to (4,0,0)
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(4.0, 0.0, 0.0);

    ConstraintChecker3D checker(meshData_);
    ConstrainedSubsegment3D subsegment;
    subsegment.nodeId1 = n0;
    subsegment.nodeId2 = n1;
    subsegment.geometryId = "test_edge";

    // Point at center of segment - definitely inside diametral sphere
    Point3D inside(2.0, 0.5, 0.0);
    // Point far from segment - outside diametral sphere
    Point3D outside(2.0, 3.0, 0.0);

    EXPECT_TRUE(checker.isSubsegmentEncroached(subsegment, inside));
    EXPECT_FALSE(checker.isSubsegmentEncroached(subsegment, outside));
}

TEST_F(ConstraintChecker3DTest, IsSubsegmentEncroachedReturnsFalseForEndpoints)
{
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(4.0, 0.0, 0.0);

    ConstraintChecker3D checker(meshData_);
    ConstrainedSubsegment3D subsegment{n0, n1, "test_edge"};

    // Endpoints are on the boundary of diametral sphere, not strictly inside
    Point3D endpoint1(0.0, 0.0, 0.0);
    Point3D endpoint2(4.0, 0.0, 0.0);

    EXPECT_FALSE(checker.isSubsegmentEncroached(subsegment, endpoint1));
    EXPECT_FALSE(checker.isSubsegmentEncroached(subsegment, endpoint2));
}

TEST_F(ConstraintChecker3DTest, IsSubsegmentEncroachedFor3DDiagonalSegment)
{
    // Diagonal segment in 3D space
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 2.0, 2.0);

    ConstraintChecker3D checker(meshData_);
    ConstrainedSubsegment3D subsegment{n0, n1, "test_edge"};

    // Midpoint - definitely inside
    Point3D inside(1.0, 1.0, 1.0);
    // Far from segment
    Point3D outside(5.0, 5.0, 5.0);

    EXPECT_TRUE(checker.isSubsegmentEncroached(subsegment, inside));
    EXPECT_FALSE(checker.isSubsegmentEncroached(subsegment, outside));
}

TEST_F(ConstraintChecker3DTest, IsSubsegmentEncroachedPointOnSphereEdge)
{
    // Segment from (0,0,0) to (2,0,0), so center is (1,0,0) and radius is 1
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);

    ConstraintChecker3D checker(meshData_);
    ConstrainedSubsegment3D subsegment{n0, n1, "test_edge"};

    // Point exactly on sphere surface (distance = radius = 1)
    Point3D onSurface(1.0, 1.0, 0.0);
    // Point just inside sphere
    Point3D justInside(1.0, 0.9, 0.0);

    // On surface should not encroach (we use strict inequality)
    EXPECT_FALSE(checker.isSubsegmentEncroached(subsegment, onSurface));
    EXPECT_TRUE(checker.isSubsegmentEncroached(subsegment, justInside));
}

// ============================================================================
// Subfacet Encroachment Tests (Equatorial Sphere)
// ============================================================================

TEST_F(ConstraintChecker3DTest, IsSubfacetEncroachedDetectsNonCoplanarPointInSphere)
{
    // Create a triangle in the XY plane
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(1.0, 2.0, 0.0);

    ConstraintChecker3D checker(meshData_);
    ConstrainedSubfacet3D subfacet{n0, n1, n2, "test_face"};

    // Point above triangle center - non-coplanar and likely inside equatorial sphere
    Point3D above(1.0, 0.5, 0.1);
    // Point far above - outside equatorial sphere
    Point3D farAbove(1.0, 0.5, 10.0);

    EXPECT_TRUE(checker.isSubfacetEncroached(subfacet, above));
    EXPECT_FALSE(checker.isSubfacetEncroached(subfacet, farAbove));
}

TEST_F(ConstraintChecker3DTest, IsSubfacetEncroachedReturnsFalseForCoplanarPoints)
{
    // Create a triangle in the XY plane (z = 0)
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(1.0, 2.0, 0.0);

    ConstraintChecker3D checker(meshData_);
    ConstrainedSubfacet3D subfacet{n0, n1, n2, "test_face"};

    // Coplanar points should NEVER encroach (per Shewchuk's definition)
    Point3D coplanarCenter(1.0, 0.67, 0.0);  // Centroid
    Point3D coplanarInside(1.0, 0.5, 0.0);   // Inside triangle
    Point3D coplanarOutside(5.0, 5.0, 0.0);  // Outside triangle but coplanar

    EXPECT_FALSE(checker.isSubfacetEncroached(subfacet, coplanarCenter));
    EXPECT_FALSE(checker.isSubfacetEncroached(subfacet, coplanarInside));
    EXPECT_FALSE(checker.isSubfacetEncroached(subfacet, coplanarOutside));
}

TEST_F(ConstraintChecker3DTest, IsSubfacetEncroachedForVertexPoints)
{
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(1.0, 2.0, 0.0);

    ConstraintChecker3D checker(meshData_);
    ConstrainedSubfacet3D subfacet{n0, n1, n2, "test_face"};

    // Triangle vertices are coplanar, so should not encroach
    Point3D v0(0.0, 0.0, 0.0);
    Point3D v1(2.0, 0.0, 0.0);
    Point3D v2(1.0, 2.0, 0.0);

    EXPECT_FALSE(checker.isSubfacetEncroached(subfacet, v0));
    EXPECT_FALSE(checker.isSubfacetEncroached(subfacet, v1));
    EXPECT_FALSE(checker.isSubfacetEncroached(subfacet, v2));
}

TEST_F(ConstraintChecker3DTest, IsSubfacetEncroachedBothSidesOfPlane)
{
    // Triangle in XY plane
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(1.0, 2.0, 0.0);

    ConstraintChecker3D checker(meshData_);
    ConstrainedSubfacet3D subfacet{n0, n1, n2, "test_face"};

    // Points on both sides of the plane, close to center
    Point3D above(1.0, 0.5, 0.1);
    Point3D below(1.0, 0.5, -0.1);

    // Both should encroach if they're inside the equatorial sphere
    EXPECT_TRUE(checker.isSubfacetEncroached(subfacet, above));
    EXPECT_TRUE(checker.isSubfacetEncroached(subfacet, below));
}

TEST_F(ConstraintChecker3DTest, IsSubfacetEncroachedEquilateralTriangle)
{
    // Equilateral triangle with known circumcenter
    double h = std::sqrt(3.0);
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(2.0, 0.0, 0.0);
    size_t n2 = addNode(1.0, h, 0.0);

    ConstraintChecker3D checker(meshData_);
    ConstrainedSubfacet3D subfacet{n0, n1, n2, "test_face"};

    // Circumcenter of equilateral triangle is at centroid
    double cx = 1.0;
    double cy = h / 3.0;

    // Point directly above circumcenter, very close
    Point3D nearCircumcenter(cx, cy, 0.01);
    // Point directly above circumcenter, far away
    Point3D farAbove(cx, cy, 5.0);

    EXPECT_TRUE(checker.isSubfacetEncroached(subfacet, nearCircumcenter));
    EXPECT_FALSE(checker.isSubfacetEncroached(subfacet, farAbove));
}

TEST_F(ConstraintChecker3DTest, IsSubfacetEncroachedArbitraryPlane)
{
    // Triangle not aligned with coordinate planes
    size_t n0 = addNode(1.0, 0.0, 0.0);
    size_t n1 = addNode(0.0, 1.0, 0.0);
    size_t n2 = addNode(0.0, 0.0, 1.0);

    ConstraintChecker3D checker(meshData_);
    ConstrainedSubfacet3D subfacet{n0, n1, n2, "test_face"};

    // Centroid of this triangle
    Point3D centroid(1.0/3.0, 1.0/3.0, 1.0/3.0);

    // Point along normal from centroid (non-coplanar)
    // Normal direction is (1,1,1) normalized
    double offset = 0.05;
    Point3D aboveCentroid(
        centroid.x() + offset,
        centroid.y() + offset,
        centroid.z() + offset
    );

    // This should encroach since it's close and non-coplanar
    EXPECT_TRUE(checker.isSubfacetEncroached(subfacet, aboveCentroid));
}

// ============================================================================
// Edge Cases and Invalid Input Tests
// ============================================================================

TEST_F(ConstraintChecker3DTest, IsSubsegmentEncroachedWithInvalidNodeIds)
{
    size_t n0 = addNode(0.0, 0.0, 0.0);
    // n1 doesn't exist (ID 999)

    ConstraintChecker3D checker(meshData_);
    ConstrainedSubsegment3D subsegment{n0, 999, "test_edge"};

    Point3D anyPoint(1.0, 1.0, 1.0);

    // Should return false for invalid node IDs
    EXPECT_FALSE(checker.isSubsegmentEncroached(subsegment, anyPoint));
}

TEST_F(ConstraintChecker3DTest, IsSubfacetEncroachedWithInvalidNodeIds)
{
    size_t n0 = addNode(0.0, 0.0, 0.0);
    size_t n1 = addNode(1.0, 0.0, 0.0);
    // n2 doesn't exist (ID 999)

    ConstraintChecker3D checker(meshData_);
    ConstrainedSubfacet3D subfacet{n0, n1, 999, "test_face"};

    Point3D anyPoint(0.5, 0.5, 0.5);

    // Should return false for invalid node IDs
    EXPECT_FALSE(checker.isSubfacetEncroached(subfacet, anyPoint));
}
