#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/General/RobustPredicates3D.h"

#include <cmath>

using namespace Meshing;

// ============================================================================
// orientationSign
// ============================================================================

TEST(RobustPredicates3DTest, OrientationSign_UnitTetrahedron_ReturnsConsistentNonzeroSign)
{
    // orientationSign's absolute sign convention (which winding counts as
    // "positive") isn't part of the contract — only that it's nonzero for a
    // genuine tetrahedron and consistent under vertex swaps (see next test).
    const Point3D p0(0.0, 0.0, 0.0);
    const Point3D p1(1.0, 0.0, 0.0);
    const Point3D p2(0.0, 1.0, 0.0);
    const Point3D p3(0.0, 0.0, 1.0);

    EXPECT_EQ(RobustPredicates3D::orientationSign(p0, p1, p2, p3), -1);
}

TEST(RobustPredicates3DTest, OrientationSign_SwappingTwoVertices_FlipsSign)
{
    const Point3D p0(0.0, 0.0, 0.0);
    const Point3D p1(1.0, 0.0, 0.0);
    const Point3D p2(0.0, 1.0, 0.0);
    const Point3D p3(0.0, 0.0, 1.0);

    EXPECT_EQ(RobustPredicates3D::orientationSign(p1, p0, p2, p3), 1);
}

TEST(RobustPredicates3DTest, OrientationSign_CoplanarPoints_ReturnsZero)
{
    const Point3D p0(0.0, 0.0, 0.0);
    const Point3D p1(1.0, 0.0, 0.0);
    const Point3D p2(0.0, 1.0, 0.0);
    const Point3D p3(1.0, 1.0, 0.0); // same z=0 plane as the other three

    EXPECT_EQ(RobustPredicates3D::orientationSign(p0, p1, p2, p3), 0);
}

// ============================================================================
// segmentCrossesTriangle (OPE-169)
// ============================================================================

TEST(RobustPredicates3DTest, SegmentCrossesTriangle_ThroughInterior_ReturnsTrue)
{
    const Point3D a(0.0, 0.0, 0.0);
    const Point3D b(1.0, 0.0, 0.0);
    const Point3D c(0.0, 1.0, 0.0);

    const Point3D p(0.2, 0.2, -1.0);
    const Point3D q(0.2, 0.2, 1.0);

    EXPECT_TRUE(RobustPredicates3D::segmentCrossesTriangle(p, q, a, b, c));
}

TEST(RobustPredicates3DTest, SegmentCrossesTriangle_PlaneCrossingOutsideTriangle_ReturnsFalse)
{
    // Crosses the triangle's plane (z=0) at (1.2, 1.2, 0), which is outside
    // the triangle (x+y > 1) even though the segment straddles the plane.
    const Point3D a(0.0, 0.0, 0.0);
    const Point3D b(1.0, 0.0, 0.0);
    const Point3D c(0.0, 1.0, 0.0);

    const Point3D p(0.2, 0.2, -1.0);
    const Point3D q(2.0, 2.0, 1.0);

    EXPECT_FALSE(RobustPredicates3D::segmentCrossesTriangle(p, q, a, b, c));
}

TEST(RobustPredicates3DTest, SegmentCrossesTriangle_BothEndpointsSameSide_ReturnsFalse)
{
    const Point3D a(0.0, 0.0, 0.0);
    const Point3D b(1.0, 0.0, 0.0);
    const Point3D c(0.0, 1.0, 0.0);

    const Point3D p(0.2, 0.2, 1.0);
    const Point3D q(0.2, 0.2, 2.0);

    EXPECT_FALSE(RobustPredicates3D::segmentCrossesTriangle(p, q, a, b, c));
}

TEST(RobustPredicates3DTest, SegmentCrossesTriangle_CoplanarSegment_ReturnsFalse)
{
    // Segment lies exactly in the triangle's own plane -- not a transversal
    // crossing, so this must not be reported as one.
    const Point3D a(0.0, 0.0, 0.0);
    const Point3D b(1.0, 0.0, 0.0);
    const Point3D c(0.0, 1.0, 0.0);

    const Point3D p(0.1, 0.1, 0.0);
    const Point3D q(0.3, 0.3, 0.0);

    EXPECT_FALSE(RobustPredicates3D::segmentCrossesTriangle(p, q, a, b, c));
}

TEST(RobustPredicates3DTest, SegmentCrossesTriangle_TouchesVertex_ReturnsFalse)
{
    // The plane crossing point coincides exactly with vertex a -- an edge
    // orientation test lands on exactly 0, which must be rejected rather
    // than treated as a match.
    const Point3D a(0.0, 0.0, 0.0);
    const Point3D b(1.0, 0.0, 0.0);
    const Point3D c(0.0, 1.0, 0.0);

    const Point3D p(0.0, 0.0, -1.0);
    const Point3D q(0.0, 0.0, 1.0);

    EXPECT_FALSE(RobustPredicates3D::segmentCrossesTriangle(p, q, a, b, c));
}

TEST(RobustPredicates3DTest, SegmentCrossesTriangle_ExtremelyLongSegment_StillExact)
{
    // A dual edge can end at the orthocentre of a nearly flat tetrahedron,
    // hundreds of units away from the triangle (OPE-169) --
    // the predicate must still resolve the crossing exactly rather than
    // drift with the endpoint's magnitude.
    const Point3D a(0.0, 0.0, 0.0);
    const Point3D b(1.0, 0.0, 0.0);
    const Point3D c(0.0, 1.0, 0.0);

    const Point3D p(0.2, 0.2, -1.0);
    const Point3D q(0.2, 0.2, 1000.0);

    EXPECT_TRUE(RobustPredicates3D::segmentCrossesTriangle(p, q, a, b, c));
}
