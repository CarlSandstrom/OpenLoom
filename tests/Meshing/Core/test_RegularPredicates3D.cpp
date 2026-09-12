#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/General/RegularPredicates3D.h"
#include "Meshing/Core/3D/General/RobustPredicates3D.h"

using namespace Meshing;

// ============================================================================
// insidePointOrthosphere — zero weight reduces exactly to insidePointCircumsphere
// ============================================================================

TEST(RegularPredicates3DTest, ZeroWeights_CenterOfRegularTetrahedron_MatchesUnweighted)
{
    const Point3D p0(1.0, 1.0, 1.0);
    const Point3D p1(1.0, -1.0, -1.0);
    const Point3D p2(-1.0, 1.0, -1.0);
    const Point3D p3(-1.0, -1.0, 1.0);
    const Point3D queryPoint(0.0, 0.0, 0.0);

    EXPECT_EQ(RegularPredicates3D::insidePointOrthosphere(p0, 0.0, p1, 0.0, p2, 0.0, p3, 0.0, queryPoint, 0.0),
             RobustPredicates3D::insidePointCircumsphere(p0, p1, p2, p3, queryPoint));
    EXPECT_TRUE(RegularPredicates3D::insidePointOrthosphere(p0, 0.0, p1, 0.0, p2, 0.0, p3, 0.0, queryPoint, 0.0));
}

TEST(RegularPredicates3DTest, ZeroWeights_FarPoint_MatchesUnweighted)
{
    const Point3D p0(1.0, 1.0, 1.0);
    const Point3D p1(1.0, -1.0, -1.0);
    const Point3D p2(-1.0, 1.0, -1.0);
    const Point3D p3(-1.0, -1.0, 1.0);
    const Point3D queryPoint(100.0, 100.0, 100.0);

    EXPECT_EQ(RegularPredicates3D::insidePointOrthosphere(p0, 0.0, p1, 0.0, p2, 0.0, p3, 0.0, queryPoint, 0.0),
             RobustPredicates3D::insidePointCircumsphere(p0, p1, p2, p3, queryPoint));
    EXPECT_FALSE(RegularPredicates3D::insidePointOrthosphere(p0, 0.0, p1, 0.0, p2, 0.0, p3, 0.0, queryPoint, 0.0));
}

TEST(RegularPredicates3DTest, ZeroWeights_MatchesUnweighted_AcrossManyPoints)
{
    // Cross-check over a scattered set of query points and both tet windings
    // to make sure the sign-vs-orientationSign comparison generalizes
    // (rather than happening to agree only for the two cases above).
    const Point3D p0(0.0, 0.0, 0.0);
    const Point3D p1(2.0, 0.0, 0.0);
    const Point3D p2(0.0, 2.0, 0.0);
    const Point3D p3(0.0, 0.0, 2.0);

    const std::vector<Point3D> queries = {
        Point3D(0.5, 0.5, 0.5), Point3D(0.9, 0.9, 0.9), Point3D(5.0, 0.0, 0.0),
        Point3D(-1.0, -1.0, -1.0), Point3D(1.0, 1.0, 1.0), Point3D(2.0, 2.0, 2.0),
    };

    for (const Point3D& queryPoint : queries)
    {
        EXPECT_EQ(RegularPredicates3D::insidePointOrthosphere(p0, 0.0, p1, 0.0, p2, 0.0, p3, 0.0, queryPoint, 0.0),
                 RobustPredicates3D::insidePointCircumsphere(p0, p1, p2, p3, queryPoint))
            << "queryPoint = " << queryPoint.transpose();
        // Flipped winding (p1, p0 swapped) must agree too -- the predicate's
        // inside/outside answer shouldn't depend on the tet's own winding.
        EXPECT_EQ(RegularPredicates3D::insidePointOrthosphere(p1, 0.0, p0, 0.0, p2, 0.0, p3, 0.0, queryPoint, 0.0),
                 RobustPredicates3D::insidePointCircumsphere(p1, p0, p2, p3, queryPoint))
            << "queryPoint = " << queryPoint.transpose();
    }
}

TEST(RegularPredicates3DTest, DegenerateTetrahedron_ReturnsFalse)
{
    const Point3D p0(0.0, 0.0, 0.0);
    const Point3D p1(1.0, 0.0, 0.0);
    const Point3D p2(0.0, 1.0, 0.0);
    const Point3D p3(1.0, 1.0, 0.0); // coplanar with p0,p1,p2

    EXPECT_FALSE(
        RegularPredicates3D::insidePointOrthosphere(p0, 0.0, p1, 0.0, p2, 0.0, p3, 0.0, Point3D(0.5, 0.5, 0.5), 0.0));
}

// ============================================================================
// insidePointOrthosphere — genuinely weighted cases
// ============================================================================

TEST(RegularPredicates3DTest, IncreasingQueryWeight_TurnsOutsidePointInside)
{
    // A point just outside the unweighted circumsphere: increasing its own
    // weight enough must flip it to "inside" the orthosphere (a heavily
    // weighted query point has a larger zone of influence -- exactly the
    // effect a protecting ball's weight needs to have on Bowyer-Watson
    // conflict detection).
    const Point3D p0(1.0, 1.0, 1.0);
    const Point3D p1(1.0, -1.0, -1.0);
    const Point3D p2(-1.0, 1.0, -1.0);
    const Point3D p3(-1.0, -1.0, 1.0);

    const Point3D queryPoint(2.0, 0.0, 0.0);
    ASSERT_FALSE(
        RegularPredicates3D::insidePointOrthosphere(p0, 0.0, p1, 0.0, p2, 0.0, p3, 0.0, queryPoint, 0.0));

    EXPECT_TRUE(
        RegularPredicates3D::insidePointOrthosphere(p0, 0.0, p1, 0.0, p2, 0.0, p3, 0.0, queryPoint, 100.0));
}

TEST(RegularPredicates3DTest, IncreasingVertexWeight_TurnsOutsidePointInside)
{
    // Increasing one of the TET's own vertex weights can turn a point that
    // was outside the unweighted circumsphere into one inside the weighted
    // orthosphere -- not always via a simple "bulges toward that vertex"
    // intuition (the orthocenter is a global equal-power solve across all 4
    // vertices, so a single weight change can shift it in a non-obvious
    // direction), so this case is a verified numeric example (cross-checked
    // against an independent linear-solve computation of the "equal power"
    // orthocenter, and against 20000 random weighted configurations) rather
    // than a hand-derived one.
    const Point3D p0(1.0, 1.0, 1.0);
    const Point3D p1(1.0, -1.0, -1.0);
    const Point3D p2(-1.0, 1.0, -1.0);
    const Point3D p3(-1.0, -1.0, 1.0);

    const Point3D queryPoint(-0.16714370728371986, -2.395792751589805, -0.3949689872772977);
    ASSERT_FALSE(
        RegularPredicates3D::insidePointOrthosphere(p0, 0.0, p1, 0.0, p2, 0.0, p3, 0.0, queryPoint, 0.0));

    EXPECT_TRUE(
        RegularPredicates3D::insidePointOrthosphere(p0, 10.0, p1, 0.0, p2, 0.0, p3, 0.0, queryPoint, 0.0));
}

TEST(RegularPredicates3DTest, DecreasingQueryWeight_KeepsInsidePointInside_ThenPushesItOut)
{
    // A large negative weight (a "shrunk" point) should eventually push a
    // previously-inside point back outside.
    const Point3D p0(1.0, 1.0, 1.0);
    const Point3D p1(1.0, -1.0, -1.0);
    const Point3D p2(-1.0, 1.0, -1.0);
    const Point3D p3(-1.0, -1.0, 1.0);

    const Point3D queryPoint(0.0, 0.0, 0.0);
    ASSERT_TRUE(
        RegularPredicates3D::insidePointOrthosphere(p0, 0.0, p1, 0.0, p2, 0.0, p3, 0.0, queryPoint, 0.0));

    EXPECT_FALSE(
        RegularPredicates3D::insidePointOrthosphere(p0, 0.0, p1, 0.0, p2, 0.0, p3, 0.0, queryPoint, -100.0));
}
