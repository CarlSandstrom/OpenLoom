#include "Common/Types.h"
#include "Geometry/3D/Base/DiscretizationSettings3D.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/BoundaryDiscretizer3D.h"
#include "Meshing/Core/3D/RCDT/CurveSegmentOperations.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "Topology/Corner3D.h"
#include "Topology/Edge3D.h"
#include "Topology/Surface3D.h"
#include "Topology/Topology3D.h"
#include <gtest/gtest.h>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

// ============================================================================
// Mock geometry classes
// ============================================================================

namespace
{

class MockCorner3D : public Geometry3D::ICorner3D
{
public:
    MockCorner3D(const std::string& id, const Point3D& point) :
        id_(id), point_(point) {}

    Point3D getPoint() const override { return point_; }
    std::string getId() const override { return id_; }

private:
    std::string id_;
    Point3D point_;
};

class MockEdge3D : public Geometry3D::IEdge3D
{
public:
    MockEdge3D(const std::string& id, const Point3D& start, const Point3D& end) :
        id_(id), start_(start), end_(end) {}

    Point3D getPoint(double t) const override { return start_ + t * (end_ - start_); }
    std::array<double, 3> getTangent(double /*t*/) const override
    {
        Point3D direction = (end_ - start_).normalized();
        return {direction.x(), direction.y(), direction.z()};
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

// ============================================================================
// Triangle fixture (3 corners, 3 edges, no surfaces)
//
//   C1 ──E12── C2
//     \        /
//    E13      E23
//       \    /
//        C3
// ============================================================================

struct TriangleFixture
{
    std::unique_ptr<Geometry3D::GeometryCollection3D> geometry;
    std::unique_ptr<Topology3D::Topology3D> topology;

    TriangleFixture()
    {
        Point3D ptC1(0.0, 0.0, 0.0);
        Point3D ptC2(1.0, 0.0, 0.0);
        Point3D ptC3(0.5, 1.0, 0.0);

        std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> corners;
        corners["C1"] = std::make_unique<MockCorner3D>("C1", ptC1);
        corners["C2"] = std::make_unique<MockCorner3D>("C2", ptC2);
        corners["C3"] = std::make_unique<MockCorner3D>("C3", ptC3);

        std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
        edges["E12"] = std::make_unique<MockEdge3D>("E12", ptC1, ptC2);
        edges["E13"] = std::make_unique<MockEdge3D>("E13", ptC1, ptC3);
        edges["E23"] = std::make_unique<MockEdge3D>("E23", ptC2, ptC3);

        std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaces;

        geometry = std::make_unique<Geometry3D::GeometryCollection3D>(
            std::move(surfaces), std::move(edges), std::move(corners));

        std::unordered_map<std::string, Topology3D::Corner3D> topoCorners;
        topoCorners.emplace("C1", Topology3D::Corner3D("C1", {"E12", "E13"}, {}));
        topoCorners.emplace("C2", Topology3D::Corner3D("C2", {"E12", "E23"}, {}));
        topoCorners.emplace("C3", Topology3D::Corner3D("C3", {"E13", "E23"}, {}));

        std::unordered_map<std::string, Topology3D::Edge3D> topoEdges;
        topoEdges.emplace("E12", Topology3D::Edge3D("E12", "C1", "C2", {}));
        topoEdges.emplace("E13", Topology3D::Edge3D("E13", "C1", "C3", {}));
        topoEdges.emplace("E23", Topology3D::Edge3D("E23", "C2", "C3", {}));

        std::unordered_map<std::string, Topology3D::Surface3D> topoSurfaces;

        topology = std::make_unique<Topology3D::Topology3D>(
            topoSurfaces, topoEdges, topoCorners);
    }
};

} // namespace

// ============================================================================
// Tests
// ============================================================================

TEST(RCDTContext, PhaseOne_SegmentParameterRangesMatchEdgeGeometry)
{
    TriangleFixture fixture;

    // 2 segments → E12 should produce segments [0.0, 0.5] and [0.5, 1.0]
    Geometry3D::DiscretizationSettings3D settings(2, 0);
    BoundaryDiscretizer3D discretizer(*fixture.geometry, *fixture.topology, settings);
    discretizer.discretize();
    auto result = discretizer.releaseDiscretizationResult();

    std::map<size_t, size_t> pointIndexToNodeIdMap;
    for (size_t i = 0; i < result->points.size(); ++i)
        pointIndexToNodeIdMap[i] = i;

    CurveSegmentManager manager;
    buildCurveSegments(manager, *fixture.topology, *fixture.geometry,
                       result->edgeIdToPointIndicesMap,
                       pointIndexToNodeIdMap,
                       result->edgeParameters);

    const auto e12Segments = manager.getSegmentsForEdge("E12");
    ASSERT_EQ(e12Segments.size(), 2u);
    EXPECT_DOUBLE_EQ(e12Segments[0].tStart, 0.0);
    EXPECT_DOUBLE_EQ(e12Segments[0].tEnd, 0.5);
    EXPECT_DOUBLE_EQ(e12Segments[1].tStart, 0.5);
    EXPECT_DOUBLE_EQ(e12Segments[1].tEnd, 1.0);
}

TEST(RCDTContext, PhaseOne_SegmentsCoverFullParameterRange)
{
    TriangleFixture fixture;

    // 3 segments → each edge has contiguous coverage from tMin=0.0 to tMax=1.0
    Geometry3D::DiscretizationSettings3D settings(3, 0);
    BoundaryDiscretizer3D discretizer(*fixture.geometry, *fixture.topology, settings);
    discretizer.discretize();
    auto result = discretizer.releaseDiscretizationResult();

    std::map<size_t, size_t> pointIndexToNodeIdMap;
    for (size_t i = 0; i < result->points.size(); ++i)
        pointIndexToNodeIdMap[i] = i;

    CurveSegmentManager manager;
    buildCurveSegments(manager, *fixture.topology, *fixture.geometry,
                       result->edgeIdToPointIndicesMap,
                       pointIndexToNodeIdMap,
                       result->edgeParameters);

    for (const std::string& edgeId : std::vector<std::string>{"E12", "E13", "E23"})
    {
        const auto segments = manager.getSegmentsForEdge(edgeId);
        ASSERT_EQ(segments.size(), 3u) << "Edge: " << edgeId;
        EXPECT_DOUBLE_EQ(segments.front().tStart, 0.0) << "Edge: " << edgeId;
        EXPECT_DOUBLE_EQ(segments.back().tEnd, 1.0) << "Edge: " << edgeId;

        for (size_t i = 0; i + 1 < segments.size(); ++i)
            EXPECT_DOUBLE_EQ(segments[i].tEnd, segments[i + 1].tStart)
                << "Edge: " << edgeId << " gap at segment " << i;
    }
}
