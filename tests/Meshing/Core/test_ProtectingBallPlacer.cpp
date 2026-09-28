#include "Meshing/Core/3D/RCDT/ProtectingBallPlacer.h"

#include "SaddleShape.h"

#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Data/3D/DiscretizationResult3D.h"
#include "Readers/OpenCascade/TopoDS_ShapeConverter.h"
#include "Topology/SeamCollection.h"
#include "Topology/Topology3D.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopoDS_Shape.hxx>

#include <algorithm>
#include <limits>
#include <numbers>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>

#include <gtest/gtest.h>

using namespace Meshing;

// ============================================================================
// ProtectingBallPlacerTest
//
// The three conditions CGAL Mesh_3's protection guarantees, and the reason
// the placer exists: our older protection broke them on every model measured
// (OPE-186), and the CGAL-style refinement path only terminates cleanly on
// protection that satisfies them.
//
//   1. No protected point is hidden by another ball: |p - q|^2 >= w_q - w_p.
//   2. Balls that are not neighbours on a curve are disjoint.
//   3. Neighbours along a curve overlap, so the whole curve is covered.
// ============================================================================

namespace
{

struct Protection
{
    DiscretizationResult3D discretization;
    std::unordered_map<size_t, double> weights;
};

Protection protect(const TopoDS_Shape& shape, const Geometry3D::DiscretizationSettings3D& settings)
{
    const Readers::TopoDS_ShapeConverter converter(shape);
    auto discretization =
        BoundaryDiscretizer3D::discretize(converter.getGeometryCollection(), converter.getTopology(), settings);
    Protection protection{std::move(*discretization), {}};
    protection.weights = ProtectingBallPlacer::place(protection.discretization, converter.getTopology(),
                                                     converter.getGeometryCollection());
    return protection;
}

std::set<std::pair<size_t, size_t>> neighbours(const DiscretizationResult3D& discretization)
{
    std::set<std::pair<size_t, size_t>> pairs;
    for (const auto& [edgeId, chain] : discretization.edgeIdToPointIndicesMap)
        for (size_t i = 0; i + 1 < chain.size(); ++i)
            pairs.insert(std::minmax(chain[i], chain[i + 1]));
    return pairs;
}

void expectCgalProtectionConditions(const Protection& protection)
{
    const auto& points = protection.discretization.points;
    const auto adjacent = neighbours(protection.discretization);
    ASSERT_FALSE(protection.weights.empty());

    for (const auto& [i, wi] : protection.weights)
        for (const auto& [j, wj] : protection.weights)
        {
            if (i >= j)
                continue;
            const double distance = (points[i] - points[j]).norm();
            EXPECT_GE(distance * distance, wj - wi) << "point " << i << " is hidden by the ball of " << j;
            EXPECT_GE(distance * distance, wi - wj) << "point " << j << " is hidden by the ball of " << i;
            if (!adjacent.count({i, j}))
                EXPECT_GE(distance, std::sqrt(wi) + std::sqrt(wj))
                    << "balls of " << i << " and " << j << " intersect but are not neighbours";
        }

    for (const auto& [a, b] : adjacent)
    {
        if (!protection.weights.count(a) || !protection.weights.count(b))
            continue;
        EXPECT_LT((points[a] - points[b]).norm(), std::sqrt(protection.weights.at(a)) + std::sqrt(protection.weights.at(b)))
            << "neighbours " << a << " and " << b << " do not overlap: the curve between them is uncovered";
    }
}

} // namespace

TEST(ProtectingBallPlacerTest, SaddleProtectionMeetsCgalConditions)
{
    expectCgalProtectionConditions(
        protect(TestSupport::buildSaddleSolid(), Geometry3D::DiscretizationSettings3D(std::nullopt, std::numbers::pi / 8.0, 2)));
}

TEST(ProtectingBallPlacerTest, CylinderProtectionMeetsCgalConditions)
{
    const TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(3.0, 8.0).Shape();
    expectCgalProtectionConditions(
        protect(cylinder, Geometry3D::DiscretizationSettings3D(std::nullopt, std::numbers::pi * 2.0 / 20.0 + 0.01, 0)));
}

TEST(ProtectingBallPlacerTest, BoxProtectionMeetsCgalConditions)
{
    const TopoDS_Shape box = BRepPrimAPI_MakeBox(4.0, 2.0, 1.0).Shape();
    expectCgalProtectionConditions(protect(box, Geometry3D::DiscretizationSettings3D(std::nullopt, std::numbers::pi / 8.0, 0)));
}

// A seam twin must share its original edge's protected points, in reverse.
TEST(ProtectingBallPlacerTest, SeamTwinReusesItsOriginalsPointsReversed)
{
    const TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(3.0, 8.0).Shape();
    const Readers::TopoDS_ShapeConverter converter(cylinder);
    auto discretization = BoundaryDiscretizer3D::discretize(
        converter.getGeometryCollection(), converter.getTopology(),
        Geometry3D::DiscretizationSettings3D(std::nullopt, std::numbers::pi * 2.0 / 20.0 + 0.01, 0));
    ProtectingBallPlacer::place(*discretization, converter.getTopology(), converter.getGeometryCollection());

    const auto& seams = converter.getTopology().getSeamCollection();
    const auto twins = seams.getSeamTwinEdgeIds();
    ASSERT_FALSE(twins.empty());
    for (const auto& twinId : twins)
    {
        auto twin = discretization->edgeIdToPointIndicesMap.at(twinId);
        std::reverse(twin.begin(), twin.end());
        EXPECT_EQ(twin, discretization->edgeIdToPointIndicesMap.at(seams.getOriginalEdgeId(twinId)));
    }
}

// The size function varies fiftyfold along the dense saddle's end parabolas:
// 1.33 at the corners, 0.028 at the apex. Interpolating between the corners
// ignored the apex and put balls of radius 0.8 there, too coarse for the
// surface to be recovered around them (OPE-186).
TEST(ProtectingBallPlacerTest, DenseSaddleBallsFollowTheSizeFunction)
{
    const Readers::TopoDS_ShapeConverter converter(TestSupport::buildSaddleSolid());
    const auto original = BoundaryDiscretizer3D::discretize(
        converter.getGeometryCollection(), converter.getTopology(),
        Geometry3D::DiscretizationSettings3D(std::nullopt, std::numbers::pi / 64.0, 2));
    auto protection = *original;
    // Measured at most 1.14 once the placer follows the size function; ~28
    // at the apex before it did.
    constexpr double MAXIMUM_RADIUS_TO_SPACING = 1.5;
    const auto weights =
        ProtectingBallPlacer::place(protection, converter.getTopology(), converter.getGeometryCollection());

    for (const auto& [edgeId, chain] : protection.edgeIdToPointIndicesMap)
    {
        const auto& originalChain = original->edgeIdToPointIndicesMap.at(edgeId);
        for (const size_t index : chain)
        {
            if (!weights.count(index))
                continue;
            const Point3D& point = protection.points[index];
            double spacing = std::numeric_limits<double>::max();
            double nearest = std::numeric_limits<double>::max();
            for (size_t i = 0; i + 1 < originalChain.size(); ++i)
            {
                const Point3D& a = original->points[originalChain[i]];
                const Point3D& b = original->points[originalChain[i + 1]];
                const double distance = std::min((point - a).norm(), (point - b).norm());
                if (distance < nearest)
                {
                    nearest = distance;
                    spacing = (b - a).norm();
                }
            }
            EXPECT_LE(std::sqrt(weights.at(index)), MAXIMUM_RADIUS_TO_SPACING * spacing)
                << "ball at " << point.transpose() << " on " << edgeId;
        }
    }
}
