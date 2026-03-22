#include <gtest/gtest.h>

#include "Common/BoundingBox2D.h"
#include "Common/Types.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ICorner3D.h"
#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/DiscretizationResult3D.h"
#include "Meshing/Core/3D/General/FacetDiscretization2DBuilder.h"
#include "Topology/SeamCollection.h"
#include "Topology/Surface3D.h"

using namespace Meshing;

namespace
{

class MockEdge3D : public Geometry3D::IEdge3D
{
public:
    MockEdge3D(double tMin, double tMax) :
        tMin_(tMin),
        tMax_(tMax)
    {
    }

    Point3D getPoint(double) const override { return Point3D::Zero(); }
    std::array<double, 3> getTangent(double) const override { return {1, 0, 0}; }
    Point3D getStartPoint() const override { return Point3D::Zero(); }
    Point3D getEndPoint() const override { return Point3D::Zero(); }
    std::pair<double, double> getParameterBounds() const override { return {tMin_, tMax_}; }
    double getLength() const override { return 1.0; }
    double getCurvature(double) const override { return 0.0; }
    std::string getId() const override { return {}; }

private:
    double tMin_;
    double tMax_;
};

class MockSurface : public Geometry3D::ISurface3D
{
public:
    MockSurface(double uMin, double uMax, double vMin, double vMax) :
        bounds_(uMin, uMax, vMin, vMax)
    {
    }

    std::array<double, 3> getNormal(double, double) const override { return {0, 0, 1}; }
    Point3D getPoint(double u, double v) const override { return Point3D(u, v, 0); }
    Common::BoundingBox2D getParameterBounds() const override { return bounds_; }
    double getGap(const Point3D& point) const override { return std::abs(point.z()); }
    Point2D projectPoint(const Point3D& point) const override { return Point2D(point.x(), point.y()); }
    std::string getId() const override { return {}; }

private:
    Common::BoundingBox2D bounds_;
};

std::unique_ptr<Geometry3D::GeometryCollection3D> makeGeometry(
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges)
{
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ISurface3D>> surfaces;
    std::unordered_map<std::string, std::unique_ptr<Geometry3D::ICorner3D>> corners;
    return std::make_unique<Geometry3D::GeometryCollection3D>(
        std::move(surfaces), std::move(edges), std::move(corners));
}

} // namespace

// ============================================================================
// Test 1: Planar surface with no seams — basic point, corner, and edge mapping
// ============================================================================

TEST(FacetDiscretization2DBuilderTest, PlanarSurface_BuildsMapsCorrectly)
{
    DiscretizationResult3D discretization3D;
    discretization3D.points = {
        Point3D(0, 0, 0), // 0 = c0
        Point3D(1, 0, 0), // 1 = c1
        Point3D(1, 1, 0), // 2 = c2
        Point3D(0, 1, 0), // 3 = c3
    };
    discretization3D.edgeParameters = {{}, {}, {}, {}};
    discretization3D.geometryIds = {{"c0"}, {"c1"}, {"c2"}, {"c3"}};
    discretization3D.cornerIdToPointIndexMap = {{"c0", 0}, {"c1", 1}, {"c2", 2}, {"c3", 3}};
    discretization3D.edgeIdToPointIndicesMap = {
        {"e0", {0, 1}},
        {"e1", {1, 2}},
        {"e2", {2, 3}},
        {"e3", {3, 0}},
    };

    MockSurface surface(0.0, 1.0, 0.0, 1.0);
    Topology3D::Surface3D topologySurface("s0", {"e0", "e1", "e2", "e3"}, {"c0", "c1", "c2", "c3"});
    Topology3D::SeamCollection seams;
    std::vector<size_t> globalPointIndices = {0, 1, 2, 3};
    auto geometry = makeGeometry({});

    FacetDiscretization2DBuilder builder(
        surface, topologySurface, seams, discretization3D, globalPointIndices,
        [](size_t index) { return index; }, *geometry);
    builder.build();

    const auto& result = builder.getDiscretization2D();

    ASSERT_EQ(result.points.size(), 4u);
    EXPECT_DOUBLE_EQ(result.points[0].x(), 0.0);
    EXPECT_DOUBLE_EQ(result.points[0].y(), 0.0);
    EXPECT_DOUBLE_EQ(result.points[1].x(), 1.0);
    EXPECT_DOUBLE_EQ(result.points[1].y(), 0.0);
    EXPECT_DOUBLE_EQ(result.points[2].x(), 1.0);
    EXPECT_DOUBLE_EQ(result.points[2].y(), 1.0);
    EXPECT_DOUBLE_EQ(result.points[3].x(), 0.0);
    EXPECT_DOUBLE_EQ(result.points[3].y(), 1.0);

    EXPECT_EQ(result.cornerIdToPointIndexMap.at("c0"), 0u);
    EXPECT_EQ(result.cornerIdToPointIndexMap.at("c1"), 1u);
    EXPECT_EQ(result.cornerIdToPointIndexMap.at("c2"), 2u);
    EXPECT_EQ(result.cornerIdToPointIndexMap.at("c3"), 3u);

    EXPECT_EQ(result.edgeIdToPointIndicesMap.at("e0"), (std::vector<size_t>{0, 1}));
    EXPECT_EQ(result.edgeIdToPointIndicesMap.at("e1"), (std::vector<size_t>{1, 2}));
    EXPECT_EQ(result.edgeIdToPointIndicesMap.at("e2"), (std::vector<size_t>{2, 3}));
    EXPECT_EQ(result.edgeIdToPointIndicesMap.at("e3"), (std::vector<size_t>{3, 0}));

    const auto& nodeIds = builder.getLocalIndexToNode3DId();
    ASSERT_EQ(nodeIds.size(), 4u);
    EXPECT_EQ(nodeIds[0], 0u);
    EXPECT_EQ(nodeIds[1], 1u);
    EXPECT_EQ(nodeIds[2], 2u);
    EXPECT_EQ(nodeIds[3], 3u);
}

// ============================================================================
// Test 2: Seam surface — seam-twin points duplicated at U + uPeriod,
//         geometry IDs on interior point remapped from original to twin
// ============================================================================

TEST(FacetDiscretization2DBuilderTest, SeamSurface_DuplicatesSeamPointsAtUPlusPeriod)
{
    // Surface with uPeriod = 2.0 (bounds 0..2 in U).
    // Two corners (c0 at bottom, c1 at top) sit at U=0.
    // One interior point on e_seam sits between them.
    DiscretizationResult3D discretization3D;
    discretization3D.points = {
        Point3D(0.0, 0.0, 0.0), // 0 = c0
        Point3D(0.0, 0.5, 0.0), // 1 = interior of e_seam
        Point3D(0.0, 1.0, 0.0), // 2 = c1
    };
    discretization3D.edgeParameters = {{}, {0.5}, {}};
    discretization3D.geometryIds = {{"c0"}, {"e_seam"}, {"c1"}};
    discretization3D.cornerIdToPointIndexMap = {{"c0", 0}, {"c1", 2}};
    discretization3D.edgeIdToPointIndicesMap = {
        {"e_seam", {0, 1, 2}},
        {"e_twin", {2, 1, 0}}, // BoundaryDiscretizer3D reverses the original seam sequence
    };

    MockSurface surface(0.0, 2.0, 0.0, 1.0);
    Topology3D::Surface3D topologySurface("s_seam", {"e_seam", "e_twin"}, {"c0", "c1"});
    Topology3D::SeamCollection seams;
    seams.addPair("e_seam", "e_twin",
                  Topology3D::SeamCollection::SeamDirection::U,
                  {2.0, 1.0}, {2.0, 0.0});

    std::unordered_map<std::string, std::unique_ptr<Geometry3D::IEdge3D>> edges;
    edges["e_seam"] = std::make_unique<MockEdge3D>(0.0, 1.0);
    auto geometry = makeGeometry(std::move(edges));

    std::vector<size_t> globalPointIndices = {0, 1, 2};
    FacetDiscretization2DBuilder builder(
        surface, topologySurface, seams, discretization3D, globalPointIndices,
        [](size_t index) { return index; }, *geometry);
    builder.build();

    const auto& result = builder.getDiscretization2D();

    // 3 original + 3 seam-shifted duplicates
    ASSERT_EQ(result.points.size(), 6u);

    // Shifted copies sit at U + 2.0 (reversed twin: c1 first, interior, c0 last)
    EXPECT_DOUBLE_EQ(result.points[3].x(), 2.0);
    EXPECT_DOUBLE_EQ(result.points[3].y(), 1.0); // c1 (reversed twin start)
    EXPECT_DOUBLE_EQ(result.points[4].x(), 2.0);
    EXPECT_DOUBLE_EQ(result.points[4].y(), 0.5); // interior
    EXPECT_DOUBLE_EQ(result.points[5].x(), 2.0);
    EXPECT_DOUBLE_EQ(result.points[5].y(), 0.0); // c0 (reversed twin end)

    // Interior point's geometry ID must be remapped from e_seam to e_twin
    ASSERT_EQ(result.geometryIds[4].size(), 1u);
    EXPECT_EQ(result.geometryIds[4][0], "e_twin");

    // Corner points' geometry IDs are unchanged
    ASSERT_EQ(result.geometryIds[5].size(), 1u);
    EXPECT_EQ(result.geometryIds[5][0], "c0");

    EXPECT_EQ(result.edgeIdToPointIndicesMap.at("e_twin"), (std::vector<size_t>{3, 4, 5}));
    EXPECT_EQ(result.edgeIdToPointIndicesMap.at("e_seam"), (std::vector<size_t>{0, 1, 2}));

    // Shifted copies map back to the same 3D node IDs as the originals
    const auto& nodeIds = builder.getLocalIndexToNode3DId();
    ASSERT_EQ(nodeIds.size(), 6u);
    EXPECT_EQ(nodeIds[3], 2u); // reversed: c1
    EXPECT_EQ(nodeIds[4], 1u); // interior
    EXPECT_EQ(nodeIds[5], 0u); // reversed: c0
}

// ============================================================================
// Test 3: Closed-circle edge whose start and end share a global index gets
//         its final endpoint remapped to the seam-shifted local index
// ============================================================================

TEST(FacetDiscretization2DBuilderTest, SeamSurface_ClosedCircleEdgeEndpointUsesShiftedIndex)
{
    // e_circle starts and ends at global index 0 (the seam corner c0).
    // After processing, the last entry in e_circle's local sequence must
    // point to the seam-shifted copy of c0, not the original.
    DiscretizationResult3D discretization3D;
    discretization3D.points = {
        Point3D(0.0, 0.0, 0.0), // 0 = c0 (seam corner)
        Point3D(1.0, 0.0, 0.0), // 1 = interior of e_circle
    };
    discretization3D.edgeParameters = {{}, {}};
    discretization3D.geometryIds = {{"c0"}, {"e_circle"}};
    discretization3D.cornerIdToPointIndexMap = {{"c0", 0}};
    discretization3D.edgeIdToPointIndicesMap = {
        {"e_twin", {0}},
        {"e_circle", {0, 1, 0}}, // closed loop: start == end == global 0
    };

    MockSurface surface(0.0, 2.0, 0.0, 1.0);
    Topology3D::Surface3D topologySurface("s_circle", {"e_twin", "e_circle"}, {"c0"});
    Topology3D::SeamCollection seams;
    seams.addPair("e_seam", "e_twin",
                  Topology3D::SeamCollection::SeamDirection::U,
                  {2.0, 0.0}, {2.0, 0.0});

    std::vector<size_t> globalPointIndices = {0, 1};
    auto geometry = makeGeometry({});

    FacetDiscretization2DBuilder builder(
        surface, topologySurface, seams, discretization3D, globalPointIndices,
        [](size_t index) { return index; }, *geometry);
    builder.build();

    const auto& result = builder.getDiscretization2D();

    // 2 original + 1 seam-shifted copy of c0
    ASSERT_EQ(result.points.size(), 3u);

    // The shifted copy of c0 is at local index 2
    EXPECT_DOUBLE_EQ(result.points[2].x(), 2.0);
    EXPECT_DOUBLE_EQ(result.points[2].y(), 0.0);

    // e_circle: start=0 (original c0), mid=1, end=2 (shifted c0)
    EXPECT_EQ(result.edgeIdToPointIndicesMap.at("e_circle"),
              (std::vector<size_t>{0, 1, 2}));
}
