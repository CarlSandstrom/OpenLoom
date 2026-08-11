#include <gtest/gtest.h>

#include "Common/Types.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/RCDT/CurveProtectionScheme.h"
#include "Meshing/Core/3D/RCDT/CurveProtectionSubdivider.h"
#include "Topology/Topology3D.h"

#include <cmath>
#include <memory>

using namespace Meshing;

namespace
{

// Straight-line edge -- same minimal mock pattern as test_BoundaryDiscretizer3D.cpp.
class MockEdge3D : public Geometry3D::IEdge3D
{
public:
    MockEdge3D(const std::string& id, const Point3D& start, const Point3D& end) :
        id_(id), start_(start), end_(end)
    {
    }

    Point3D getPoint(double t) const override { return start_ + t * (end_ - start_); }
    std::array<double, 3> getTangent(double /*t*/) const override
    {
        const Point3D dir = (end_ - start_).normalized();
        return {dir.x(), dir.y(), dir.z()};
    }
    Point3D getStartPoint() const override { return start_; }
    Point3D getEndPoint() const override { return end_; }
    std::pair<double, double> getParameterBounds() const override { return {0.0, 1.0}; }
    double getLength() const override { return (end_ - start_).norm(); }
    double getParameterAtArcLengthFraction(double tStart, double tEnd, double fraction) const override
    {
        return tStart + fraction * (tEnd - tStart);
    }
    double getCurvature(double /*t*/) const override { return 0.0; }
    std::string getId() const override { return id_; }

private:
    std::string id_;
    Point3D start_;
    Point3D end_;
};

double radiusOf(const std::unordered_map<size_t, double>& weights, size_t point)
{
    const auto it = weights.find(point);
    return it == weights.end() ? 0.0 : std::sqrt(it->second);
}

// Empty topology (no seams) is sufficient for CurveProtectionSubdivider: it
// only ever calls getSeamCollection(), never looks up a corner/edge/surface
// entity by ID.
Topology3D::Topology3D emptyTopology()
{
    return Topology3D::Topology3D({}, {}, {});
}

} // namespace

TEST(CurveProtectionSubdividerTest, NoConflict_LeavesDiscretizationUnchanged)
{
    // A single well-separated curve: CurveProtectionScheme alone already
    // satisfies property 1, so the subdivider shouldn't add anything.
    DiscretizationResult3D discretization;
    discretization.points = {Point3D(0.0, 0.0, 0.0), Point3D(1.0, 0.0, 0.0), Point3D(2.0, 0.0, 0.0)};
    discretization.edgeParameters = {{}, {0.5}, {}};
    discretization.geometryIds = {{"edgeA"}, {"edgeA"}, {"edgeA"}};
    discretization.cornerIdToPointIndexMap = {{"C0", 0}, {"C1", 2}};
    discretization.edgeIdToPointIndicesMap = {{"edgeA", {0, 1, 2}}};

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    edges["edgeA"] = std::make_unique<MockEdge3D>("edgeA", Point3D(0.0, 0.0, 0.0), Point3D(2.0, 0.0, 0.0));
    Geometry3D::GeometryCollection3D geometry({}, std::move(edges), {});
    const auto topology = emptyTopology();

    const size_t pointCountBefore = discretization.points.size();
    const auto weights = CurveProtectionSubdivider::subdivide(discretization, topology, geometry, 1e-6);

    EXPECT_EQ(discretization.points.size(), pointCountBefore);
    EXPECT_TRUE(CurveProtectionScheme::findUnresolvedSegments(discretization.edgeIdToPointIndicesMap, weights,
                                                              discretization.points)
                   .empty());
}

TEST(CurveProtectionSubdividerTest, TightCornerFarFromNaturalScale_InsertsPointsAndResolvesTheGap)
{
    // curveA: corner CA0 --(length 10)-- P1 --(length 10)-- corner CA1.
    // curveB: corner CB0, positioned 0.1 from CA0, forces CA0's disjointness
    // clamp small. curveC: corner CC0, positioned 0.1 from P1, forces P1's
    // clamp small too -- so neither endpoint of the CA0-P1 segment can rely
    // on the other's compensation; a two-point resize alone can't bridge a
    // segment this long from radii this small (see the redesign notes in
    // RCDTMesher.cpp / CurveProtectionScheme.cpp).
    DiscretizationResult3D discretization;
    discretization.points = {
        Point3D(0.0, 0.0, 0.0),   // 0: CA0
        Point3D(10.0, 0.0, 0.0),  // 1: P1 (curveA interior)
        Point3D(20.0, 0.0, 0.0),  // 2: CA1
        Point3D(0.0, 0.1, 0.0),   // 3: CB0
        Point3D(0.0, 5.0, 0.0),   // 4: CB1
        Point3D(10.0, 0.1, 0.0),  // 5: CC0
        Point3D(10.0, 5.0, 0.0),  // 6: CC1
    };
    discretization.edgeParameters = {{}, {0.5}, {}, {}, {}, {}, {}};
    discretization.geometryIds = {
        {"curveA"}, {"curveA"}, {"curveA"}, {"curveB"}, {"curveB"}, {"curveC"}, {"curveC"},
    };
    discretization.cornerIdToPointIndexMap = {
        {"CA0", 0}, {"CA1", 2}, {"CB0", 3}, {"CB1", 4}, {"CC0", 5}, {"CC1", 6},
    };
    discretization.edgeIdToPointIndicesMap = {
        {"curveA", {0, 1, 2}},
        {"curveB", {3, 4}},
        {"curveC", {5, 6}},
    };

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    edges["curveA"] = std::make_unique<MockEdge3D>("curveA", discretization.points[0], discretization.points[2]);
    edges["curveB"] = std::make_unique<MockEdge3D>("curveB", discretization.points[3], discretization.points[4]);
    edges["curveC"] = std::make_unique<MockEdge3D>("curveC", discretization.points[5], discretization.points[6]);
    Geometry3D::GeometryCollection3D geometry({}, std::move(edges), {});
    const auto topology = emptyTopology();

    const size_t pointCountBefore = discretization.points.size();
    const auto weights = CurveProtectionSubdivider::subdivide(discretization, topology, geometry, 1e-6);

    EXPECT_GT(discretization.points.size(), pointCountBefore);

    const auto unresolved = CurveProtectionScheme::findUnresolvedSegments(discretization.edgeIdToPointIndicesMap,
                                                                          weights, discretization.points);
    EXPECT_TRUE(unresolved.empty());

    // Every point on curveA (including newly-inserted ones) must still lie
    // exactly on the true curve (the x-axis segment from (0,0,0) to (20,0,0)).
    for (size_t pointIndex : discretization.edgeIdToPointIndicesMap.at("curveA"))
    {
        const Point3D& p = discretization.points[pointIndex];
        EXPECT_NEAR(p.y(), 0.0, 1e-9);
        EXPECT_NEAR(p.z(), 0.0, 1e-9);
        EXPECT_GE(p.x(), -1e-9);
        EXPECT_LE(p.x(), 20.0 + 1e-9);
    }
}

TEST(CurveProtectionSubdividerTest, GapNarrowerThanMinimumEdgeLength_LeavesItUnresolvedRatherThanLooping)
{
    // Same tight-corner shape as the previous test, but with a minimum edge
    // length larger than the gap itself: no split can ever be attempted, so
    // the subdivider must terminate immediately rather than loop forever
    // trying (and always declining) to close it.
    DiscretizationResult3D discretization;
    discretization.points = {
        Point3D(0.0, 0.0, 0.0), Point3D(10.0, 0.0, 0.0), Point3D(20.0, 0.0, 0.0),
        Point3D(0.0, 0.1, 0.0), Point3D(0.0, 5.0, 0.0),
    };
    discretization.edgeParameters = {{}, {0.5}, {}, {}, {}};
    discretization.geometryIds = {{"curveA"}, {"curveA"}, {"curveA"}, {"curveB"}, {"curveB"}};
    discretization.cornerIdToPointIndexMap = {{"CA0", 0}, {"CA1", 2}, {"CB0", 3}, {"CB1", 4}};
    discretization.edgeIdToPointIndicesMap = {
        {"curveA", {0, 1, 2}},
        {"curveB", {3, 4}},
    };

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    edges["curveA"] = std::make_unique<MockEdge3D>("curveA", discretization.points[0], discretization.points[2]);
    edges["curveB"] = std::make_unique<MockEdge3D>("curveB", discretization.points[3], discretization.points[4]);
    Geometry3D::GeometryCollection3D geometry({}, std::move(edges), {});
    const auto topology = emptyTopology();

    // Minimum edge length far larger than the 0.1-unit gap the corner's
    // clamp forces -- every candidate split is rejected immediately.
    const auto weights = CurveProtectionSubdivider::subdivide(discretization, topology, geometry, 100.0);

    // Terminates (this line is reached at all) with the violation still
    // present rather than resolved.
    const auto unresolved = CurveProtectionScheme::findUnresolvedSegments(discretization.edgeIdToPointIndicesMap,
                                                                          weights, discretization.points);
    EXPECT_FALSE(unresolved.empty());
}
