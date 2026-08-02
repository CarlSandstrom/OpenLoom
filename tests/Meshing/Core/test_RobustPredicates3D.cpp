#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/RobustPredicates3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"

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
// insidePointCircumsphere — well-conditioned cases
// ============================================================================

TEST(RobustPredicates3DTest, InsidePointCircumsphere_CenterOfRegularTetrahedron_IsInside)
{
    // Regular tetrahedron centered at the origin (alternating cube vertices).
    const Point3D p0(1.0, 1.0, 1.0);
    const Point3D p1(1.0, -1.0, -1.0);
    const Point3D p2(-1.0, 1.0, -1.0);
    const Point3D p3(-1.0, -1.0, 1.0);

    EXPECT_TRUE(RobustPredicates3D::insidePointCircumsphere(p0, p1, p2, p3, Point3D(0.0, 0.0, 0.0)));
}

TEST(RobustPredicates3DTest, InsidePointCircumsphere_FarPoint_IsOutside)
{
    const Point3D p0(1.0, 1.0, 1.0);
    const Point3D p1(1.0, -1.0, -1.0);
    const Point3D p2(-1.0, 1.0, -1.0);
    const Point3D p3(-1.0, -1.0, 1.0);

    EXPECT_FALSE(RobustPredicates3D::insidePointCircumsphere(p0, p1, p2, p3, Point3D(100.0, 100.0, 100.0)));
}

TEST(RobustPredicates3DTest, InsidePointCircumsphere_PointExactlyOnSphere_IsInside)
{
    // Inclusive on the boundary, matching what Bowyer-Watson needs (see
    // RobustPredicates3D.cpp): a tetrahedron's own vertices are exactly on
    // its circumsphere by definition — an exact-in-floating-point on-sphere
    // case (unlike e.g. sqrt(3)*(1,0,0), whose coordinates already carry
    // sqrt(3)'s own rounding error and so aren't reliably exactly on the
    // sphere).
    const Point3D p0(1.0, 1.0, 1.0);
    const Point3D p1(1.0, -1.0, -1.0);
    const Point3D p2(-1.0, 1.0, -1.0);
    const Point3D p3(-1.0, -1.0, 1.0);

    EXPECT_TRUE(RobustPredicates3D::insidePointCircumsphere(p0, p1, p2, p3, p0));
}

TEST(RobustPredicates3DTest, InsidePointCircumsphere_MatchesLinearSolveApproach_WellConditionedTet)
{
    // Cross-check against the existing (linear-solve-based) circumsphere test for a
    // well-conditioned tetrahedron, where both approaches should agree.
    MeshData3D meshData;
    MeshMutator3D mutator(meshData);
    const size_t n0 = mutator.addNode(Point3D(0.0, 0.0, 0.0));
    const size_t n1 = mutator.addNode(Point3D(2.0, 0.0, 0.0));
    const size_t n2 = mutator.addNode(Point3D(0.0, 2.0, 0.0));
    const size_t n3 = mutator.addNode(Point3D(0.0, 0.0, 2.0));
    mutator.addElement(std::make_unique<TetrahedralElement>(std::array<size_t, 4>{n0, n1, n2, n3}));

    const ElementGeometry3D elementGeometry(meshData);
    const TetrahedralElement tet(std::array<size_t, 4>{n0, n1, n2, n3});
    const auto sphere = elementGeometry.computeCircumscribingSphere(tet);
    ASSERT_TRUE(sphere.has_value());

    const Point3D inside = sphere->center;
    const Point3D outside = sphere->center + Point3D(sphere->radius * 2.0, 0.0, 0.0);

    EXPECT_TRUE(RobustPredicates3D::insidePointCircumsphere(
        Point3D(0.0, 0.0, 0.0), Point3D(2.0, 0.0, 0.0), Point3D(0.0, 2.0, 0.0), Point3D(0.0, 0.0, 2.0), inside));
    EXPECT_FALSE(RobustPredicates3D::insidePointCircumsphere(
        Point3D(0.0, 0.0, 0.0), Point3D(2.0, 0.0, 0.0), Point3D(0.0, 2.0, 0.0), Point3D(0.0, 0.0, 2.0), outside));
}

// ============================================================================
// insidePointCircumsphere — near-degenerate case (OPE-159/OPE-138 regression)
//
// A nearly-flat tetrahedron: three points exactly on z=0, a fourth lifted by
// only 1e-10. ElementGeometry3D::computeCircumscribingSphere solves a nearly
// singular 3x3 system for this shape — on the torus this produced circumsphere
// radii in the hundreds of thousands (mesh extent ~13 units), which then
// falsely flagged distant, unrelated points as being inside the sphere and
// corrupted the Bowyer-Watson conflict search. The robust determinant-sign
// predicate must not reproduce that failure.
// ============================================================================

TEST(RobustPredicates3DTest, InsidePointCircumsphere_NearDegenerateTet_DistantPointIsOutside)
{
    const Point3D p0(0.0, 0.0, 0.0);
    const Point3D p1(1.0, 0.0, 0.0);
    const Point3D p2(0.0, 1.0, 0.0);
    const Point3D p3(0.5, 0.5, 1e-10); // barely off the p0,p1,p2 plane

    const Point3D distant(100.0, 100.0, 100.0);
    EXPECT_FALSE(RobustPredicates3D::insidePointCircumsphere(p0, p1, p2, p3, distant));
}

TEST(RobustPredicates3DTest, InsidePointCircumsphere_NearDegenerateTet_OwnCentroidIsInside)
{
    const Point3D p0(0.0, 0.0, 0.0);
    const Point3D p1(1.0, 0.0, 0.0);
    const Point3D p2(0.0, 1.0, 0.0);
    const Point3D p3(0.5, 0.5, 1e-10);

    // A tetrahedron is convex and inscribed in its own circumsphere (all four
    // vertices lie on it), so its centroid is guaranteed to lie strictly
    // inside — true for any non-degenerate tet, however flat. Confirms the
    // fix doesn't degrade into "always outside" as a trivial way to dodge the
    // old bug.
    const Point3D centroid = (p0 + p1 + p2 + p3) / 4.0;
    EXPECT_TRUE(RobustPredicates3D::insidePointCircumsphere(p0, p1, p2, p3, centroid));
}
