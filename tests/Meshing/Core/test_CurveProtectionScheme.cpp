#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Meshing/Core/3D/RCDT/CurveProtectionScheme.h"

#include <algorithm>
#include <cmath>

using namespace Meshing;

namespace
{

double radiusOf(const std::unordered_map<size_t, double>& weights, size_t point)
{
    return std::sqrt(weights.at(point));
}

} // namespace

// ============================================================================
// Single straight curve, uniform sampling: 0 (corner) -- 1 -- 2 -- 3 (corner)
// ============================================================================

TEST(CurveProtectionSchemeTest, UniformCurve_EveryPointGetsAWeight)
{
    const std::vector<Point3D> points = {
        Point3D(0.0, 0.0, 0.0),
        Point3D(1.0, 0.0, 0.0),
        Point3D(2.0, 0.0, 0.0),
        Point3D(3.0, 0.0, 0.0),
    };
    const std::map<std::string, std::vector<size_t>> edges = {{"edgeA", {0, 1, 2, 3}}};
    const std::unordered_set<size_t> corners = {0, 3};

    const auto weights = CurveProtectionScheme::computeWeights(edges, corners, points);

    for (size_t p : {0u, 1u, 2u, 3u})
        EXPECT_TRUE(weights.contains(p)) << "point " << p;
}

TEST(CurveProtectionSchemeTest, UniformCurve_ConsecutiveBallsOverlap)
{
    // Property 1: for every consecutive pair, radii must sum to more than
    // the segment length between them.
    const std::vector<Point3D> points = {
        Point3D(0.0, 0.0, 0.0),
        Point3D(1.0, 0.0, 0.0),
        Point3D(2.0, 0.0, 0.0),
        Point3D(3.0, 0.0, 0.0),
    };
    const std::map<std::string, std::vector<size_t>> edges = {{"edgeA", {0, 1, 2, 3}}};
    const std::unordered_set<size_t> corners = {0, 3};

    const auto weights = CurveProtectionScheme::computeWeights(edges, corners, points);

    for (size_t k = 0; k + 1 < points.size(); ++k)
    {
        const double distance = (points[k] - points[k + 1]).norm();
        EXPECT_GT(radiusOf(weights, k) + radiusOf(weights, k + 1), distance) << "segment " << k << "-" << (k + 1);
    }
}

TEST(CurveProtectionSchemeTest, UniformCurve_CornersAreSmallerThanInteriorPoints)
{
    // Corners get "strong" (smaller) balls that interior points shrink
    // toward -- not the reverse.
    const std::vector<Point3D> points = {
        Point3D(0.0, 0.0, 0.0),
        Point3D(1.0, 0.0, 0.0),
        Point3D(2.0, 0.0, 0.0),
        Point3D(3.0, 0.0, 0.0),
    };
    const std::map<std::string, std::vector<size_t>> edges = {{"edgeA", {0, 1, 2, 3}}};
    const std::unordered_set<size_t> corners = {0, 3};

    const auto weights = CurveProtectionScheme::computeWeights(edges, corners, points);

    EXPECT_LT(radiusOf(weights, 0), radiusOf(weights, 1));
    EXPECT_LT(radiusOf(weights, 3), radiusOf(weights, 2));
}

// ============================================================================
// Non-uniform sampling: a curve with a sudden jump in segment length must
// still satisfy property 1 (this is what motivates using max(), not min() or
// an average, of the two adjacent segments -- see class comment).
// ============================================================================

TEST(CurveProtectionSchemeTest, NonUniformCurve_ConsecutiveBallsStillOverlap)
{
    // Segment lengths: 0.1, 0.1, 10.0, 0.1, 0.1 -- a sharp density change
    // right in the middle, like an angle-based discretization can produce.
    const std::vector<Point3D> points = {
        Point3D(0.0, 0.0, 0.0),
        Point3D(0.1, 0.0, 0.0),
        Point3D(0.2, 0.0, 0.0),
        Point3D(10.2, 0.0, 0.0),
        Point3D(10.3, 0.0, 0.0),
        Point3D(10.4, 0.0, 0.0),
    };
    const std::map<std::string, std::vector<size_t>> edges = {{"edgeA", {0, 1, 2, 3, 4, 5}}};
    const std::unordered_set<size_t> corners = {0, 5};

    const auto weights = CurveProtectionScheme::computeWeights(edges, corners, points);

    for (size_t k = 0; k + 1 < points.size(); ++k)
    {
        const double distance = (points[k] - points[k + 1]).norm();
        EXPECT_GT(radiusOf(weights, k) + radiusOf(weights, k + 1), distance) << "segment " << k << "-" << (k + 1);
    }
}

// ============================================================================
// Two unrelated curves passing close to each other: property 2 (disjointness)
// ============================================================================

TEST(CurveProtectionSchemeTest, TwoNearbyUnrelatedCurves_BallsDoNotOverlap)
{
    // Curve A: (0,0,0) -- (1,0,0) -- (2,0,0), corners 0 and 2.
    // Curve B: (1,0.05,0) -- (1,1,0), corners 3 and 4 -- point 3 passes
    // within 0.05 of curve A's midpoint (point 1), much closer than curve
    // A's own 1.0 sampling density would otherwise suggest.
    const std::vector<Point3D> points = {
        Point3D(0.0, 0.0, 0.0),
        Point3D(1.0, 0.0, 0.0),
        Point3D(2.0, 0.0, 0.0),
        Point3D(1.0, 0.05, 0.0),
        Point3D(1.0, 1.0, 0.0),
    };
    const std::map<std::string, std::vector<size_t>> edges = {
        {"edgeA", {0, 1, 2}},
        {"edgeB", {3, 4}},
    };
    const std::unordered_set<size_t> corners = {0, 2, 3, 4};

    const auto weights = CurveProtectionScheme::computeWeights(edges, corners, points);

    // Point 1 (curve A) and point 3 (curve B) are unrelated (different
    // curves, not chain-adjacent) -- their balls must not overlap even
    // though they're very close together.
    const double distance13 = (points[1] - points[3]).norm();
    EXPECT_LE(radiusOf(weights, 1) + radiusOf(weights, 3), distance13);
}

TEST(CurveProtectionSchemeTest, TwoDistantUnrelatedCurves_ClampDoesNotShrinkBelowConnectivityNeed)
{
    // Sanity check that the disjointness clamp is a no-op when nothing is
    // actually nearby: two curves far apart from each other should size
    // exactly as the single-curve tests do.
    const std::vector<Point3D> points = {
        Point3D(0.0, 0.0, 0.0),
        Point3D(1.0, 0.0, 0.0),
        Point3D(2.0, 0.0, 0.0),
        Point3D(0.0, 100.0, 0.0),
        Point3D(1.0, 100.0, 0.0),
        Point3D(2.0, 100.0, 0.0),
    };
    const std::map<std::string, std::vector<size_t>> edges = {
        {"edgeA", {0, 1, 2}},
        {"edgeB", {3, 4, 5}},
    };
    const std::unordered_set<size_t> corners = {0, 2, 3, 5};

    const auto weights = CurveProtectionScheme::computeWeights(edges, corners, points);

    for (size_t k = 0; k + 1 < 3; ++k)
        EXPECT_GT(radiusOf(weights, k) + radiusOf(weights, k + 1), (points[k] - points[k + 1]).norm());
    for (size_t k = 3; k + 1 < 6; ++k)
        EXPECT_GT(radiusOf(weights, k) + radiusOf(weights, k + 1), (points[k] - points[k + 1]).norm());
}

// ============================================================================
// Corner shared by multiple curves: sized from the shortest incident step.
// ============================================================================

TEST(CurveProtectionSchemeTest, CornerSharedByMultipleCurves_UsesShortestIncidentStep)
{
    // Three edges meeting at corner 0: first steps 1.0, 0.1, 1.0 -- the
    // corner's radius must be driven by the shortest (0.1), not an average
    // or the longest.
    const std::vector<Point3D> points = {
        Point3D(0.0, 0.0, 0.0), // corner
        Point3D(1.0, 0.0, 0.0), // edgeA far endpoint
        Point3D(0.0, 0.1, 0.0), // edgeB far endpoint (short)
        Point3D(0.0, 0.0, 1.0), // edgeC far endpoint
    };
    const std::map<std::string, std::vector<size_t>> edges = {
        {"edgeA", {0, 1}},
        {"edgeB", {0, 2}},
        {"edgeC", {0, 3}},
    };
    const std::unordered_set<size_t> corners = {0, 1, 2, 3};

    const auto weights = CurveProtectionScheme::computeWeights(edges, corners, points);

    EXPECT_NEAR(radiusOf(weights, 0), 0.3 * 0.1, 1e-12);
}

// ============================================================================
// findUnresolvedSegments
// ============================================================================

TEST(CurveProtectionSchemeTest, FindUnresolvedSegments_WellSeparatedCurve_ReportsNothing)
{
    const std::vector<Point3D> points = {
        Point3D(0.0, 0.0, 0.0),
        Point3D(1.0, 0.0, 0.0),
        Point3D(2.0, 0.0, 0.0),
        Point3D(3.0, 0.0, 0.0),
    };
    const std::map<std::string, std::vector<size_t>> edges = {{"edgeA", {0, 1, 2, 3}}};
    const std::unordered_set<size_t> corners = {0, 3};

    const auto weights = CurveProtectionScheme::computeWeights(edges, corners, points);
    const auto unresolved = CurveProtectionScheme::findUnresolvedSegments(edges, weights, points);

    EXPECT_TRUE(unresolved.empty());
}

TEST(CurveProtectionSchemeTest, FindUnresolvedSegments_NearbyUnrelatedCurve_ReportsTheGap)
{
    // Same construction as TwoNearbyUnrelatedCurves_BallsDoNotOverlap: point
    // 1 (curve A) and point 3 (curve B) are close enough that curve A's own
    // 0-1 segment can't be bridged once point 1 is clamped for disjointness.
    const std::vector<Point3D> points = {
        Point3D(0.0, 0.0, 0.0),
        Point3D(1.0, 0.0, 0.0),
        Point3D(2.0, 0.0, 0.0),
        Point3D(1.0, 0.05, 0.0),
        Point3D(1.0, 1.0, 0.0),
    };
    const std::map<std::string, std::vector<size_t>> edges = {
        {"edgeA", {0, 1, 2}},
        {"edgeB", {3, 4}},
    };
    const std::unordered_set<size_t> corners = {0, 2, 3, 4};

    const auto weights = CurveProtectionScheme::computeWeights(edges, corners, points);
    const auto unresolved = CurveProtectionScheme::findUnresolvedSegments(edges, weights, points);

    ASSERT_FALSE(unresolved.empty());
    EXPECT_TRUE(std::any_of(unresolved.begin(), unresolved.end(),
                            [](const auto& segment)
                            { return segment.edgeId == "edgeA"; }));
}

TEST(CurveProtectionSchemeTest, FindUnresolvedSegments_MissingWeightTreatedAsZeroRadius)
{
    // A point absent from the weights map (e.g. one a caller hasn't sized
    // yet) is treated as radius 0, not as an error.
    const std::vector<Point3D> points = {Point3D(0.0, 0.0, 0.0), Point3D(1.0, 0.0, 0.0)};
    const std::map<std::string, std::vector<size_t>> edges = {{"edgeA", {0, 1}}};
    const std::unordered_map<size_t, double> weights = {{0, 0.01}}; // point 1 absent

    const auto unresolved = CurveProtectionScheme::findUnresolvedSegments(edges, weights, points);

    ASSERT_EQ(unresolved.size(), 1u);
    EXPECT_EQ(unresolved[0].nodeId1, 0u);
    EXPECT_EQ(unresolved[0].nodeId2, 1u);
}
