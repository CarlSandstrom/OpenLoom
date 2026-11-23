#include <gtest/gtest.h>

#include "Meshing/Core/2D/Periodic/TwinManager2D.h"
#include "Common/Types.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"
#include "Meshing/Core/2D/General/BoundaryDiscretizer2D.h"
#include "Meshing/Core/2D/Periodic/BoundarySplitSynchronizer2D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/General/MeshOperations2D.h"
#include "Meshing/Core/2D/General/MeshVerifier2D.h"
#include "Meshing/Core/2D/General/MeshingContext2D.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/MeshQualitySettings2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/Base/TriangleElement.h"
#include "Topology2D/Topology2D.h"

#include <Geom2d_Circle.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt2d.hxx>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

namespace
{

// Temporarily forces an environment variable to a value, restoring whatever
// was previously set (or unset) on destruction. A plain setenv/unsetenv pair
// would clobber a value the invoking shell had already set (e.g. a caller
// running the whole suite under CHECK_MESH_EACH_ITERATION=1), silently
// disabling that check for every test that runs afterward in the same process.
class ScopedEnvironmentOverride
{
public:
    ScopedEnvironmentOverride(const std::string& name, const std::string& value) :
        name_(name)
    {
        const char* previous = std::getenv(name_.c_str());
        hadPreviousValue_ = previous != nullptr;
        if (hadPreviousValue_)
            previousValue_ = previous;
        setenv(name_.c_str(), value.c_str(), 1);
    }

    ~ScopedEnvironmentOverride()
    {
        if (hadPreviousValue_)
            setenv(name_.c_str(), previousValue_.c_str(), 1);
        else
            unsetenv(name_.c_str());
    }

private:
    std::string name_;
    bool hadPreviousValue_;
    std::string previousValue_;
};

void addSquare(Geometry2D::GeometryCollection2D& geometry,
               std::unordered_map<std::string, Topology2D::Corner2D>& topologyCorners,
               std::unordered_map<std::string, Topology2D::Edge2D>& topologyEdges,
               std::vector<std::string>& edgeLoop,
               const std::string& prefix,
               double size,
               Point2D origin = Point2D(0.0, 0.0))
{
    std::vector<Point2D> corners = {
        origin,
        Point2D(origin.x() + size, origin.y()),
        Point2D(origin.x() + size, origin.y() + size),
        Point2D(origin.x(), origin.y() + size)};

    for (size_t i = 0; i < 4; ++i)
    {
        std::string cornerId = prefix + "_c" + std::to_string(i);
        std::string edgeId = prefix + "_e" + std::to_string(i);
        std::string previousEdgeId = prefix + "_e" + std::to_string((i + 3) % 4);

        geometry.addCorner(std::make_unique<Geometry2D::Corner2D>(cornerId, corners[i]));
        geometry.addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
            edgeId, corners[i], corners[(i + 1) % 4]));

        topologyCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, previousEdgeId}));
        topologyEdges.emplace(edgeId, Topology2D::Edge2D(
                                          edgeId, cornerId, prefix + "_c" + std::to_string((i + 1) % 4)));
        edgeLoop.push_back(edgeId);
    }
}

void addCircularEdge(Geometry2D::GeometryCollection2D& geometry,
                     std::unordered_map<std::string, Topology2D::Corner2D>& topologyCorners,
                     std::unordered_map<std::string, Topology2D::Edge2D>& topologyEdges,
                     const std::string& prefix,
                     double centerX,
                     double centerY,
                     double radius,
                     size_t numberOfSegments = 8)
{
    gp_Pnt2d center(centerX, centerY);
    gp_Ax2d axis(center, gp_Dir2d(1.0, 0.0));
    gp_Circ2d circle(axis, radius);

    for (size_t i = 0; i < numberOfSegments; ++i)
    {
        double angle = 2.0 * M_PI * i / numberOfSegments;
        double x = centerX + radius * std::cos(angle);
        double y = centerY + radius * std::sin(angle);

        std::string cornerId = prefix + "_c" + std::to_string(i);
        std::string edgeId = prefix + "_e" + std::to_string(i);
        std::string previousEdgeId = prefix + "_e" + std::to_string((i + numberOfSegments - 1) % numberOfSegments);

        geometry.addCorner(std::make_unique<Geometry2D::OpenCascade2DCorner>(gp_Pnt2d(x, y), cornerId));
        topologyCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, previousEdgeId}));
    }

    for (size_t i = 0; i < numberOfSegments; ++i)
    {
        double startAngle = 2.0 * M_PI * i / numberOfSegments;
        double endAngle = 2.0 * M_PI * (i + 1) / numberOfSegments;

        Handle(Geom2d_Circle) circleGeometry = new Geom2d_Circle(circle);
        Handle(Geom2d_TrimmedCurve) arc = new Geom2d_TrimmedCurve(circleGeometry, startAngle, endAngle);

        std::string edgeId = prefix + "_e" + std::to_string(i);
        std::string startCornerId = prefix + "_c" + std::to_string(i);
        std::string endCornerId = prefix + "_c" + std::to_string((i + 1) % numberOfSegments);

        geometry.addEdge(std::make_unique<Geometry2D::OpenCascade2DEdge>(arc, edgeId));
        topologyEdges.emplace(edgeId, Topology2D::Edge2D(edgeId, startCornerId, endCornerId));
    }
}

} // namespace

// Regression test: splitting a curved constraint (arc) that borders a hole must
// not produce CW-wound triangles.
//
// Root cause: splitTrianglesAtEdge placed the arc midpoint (which lies on the
// circle boundary, not on the chord) into hole-interior triangles. The midpoint
// falls geometrically outside those triangles, producing CW sub-triangles.
// Fix (commit 8da8c8c): skip hole-interior triangles in splitTrianglesAtEdge.
TEST(ShewchukRefiner2D, TwoCircularHoles_CurvedConstraintSplitProducesValidMesh)
{
    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topologyEdges;

    std::vector<std::string> outerEdgeLoop;
    addSquare(*geometry, topologyCorners, topologyEdges, outerEdgeLoop, "outer", 10.0);

    const size_t numberOfSegments = 8;
    addCircularEdge(*geometry, topologyCorners, topologyEdges, "hole1", 4.0, 5.0, 0.95, numberOfSegments);
    addCircularEdge(*geometry, topologyCorners, topologyEdges, "hole2", 6.0, 5.0, 1.0, numberOfSegments);

    std::vector<std::string> hole1Loop, hole2Loop;
    for (size_t i = 0; i < numberOfSegments; ++i)
    {
        hole1Loop.push_back("hole1_e" + std::to_string(i));
        hole2Loop.push_back("hole2_e" + std::to_string(i));
    }

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topologyCorners, topologyEdges, outerEdgeLoop,
        std::vector<std::vector<std::string>>{hole1Loop, hole2Loop});

    MeshingContext2D context(std::move(geometry), std::move(topology));

    BoundaryDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D::triangulate(context, discretization);

    // Enable per-iteration mesh checking so that a transient CW triangle
    // (which would later be cleaned up by exterior-triangle removal) is caught
    // immediately and surfaces as an exception rather than silently surviving.
    ScopedEnvironmentOverride checkMeshEachIteration("CHECK_MESH_EACH_ITERATION", "1");

    ShewchukRefiner2D refiner(context, Meshing::MeshQualitySettings2D{});
    ASSERT_NO_THROW(refiner.refine());
}

// Regression test: splitting a curved constraint (arc) used as an internal
// constraint (not a hole boundary) must not produce overlapping triangles.
//
// Root cause: splitTrianglesAtEdge was called even when the arc midpoint lies
// off the chord. For domain-interior triangles, the off-chord midpoint is
// outside one of the two adjacent triangles — the resulting sub-triangles
// overlap those from the other triangle.
// Fix (commit 71bb20a): detect off-chord midpoints and route them through
// Bowyer-Watson instead of splitTrianglesAtEdge.
TEST(ShewchukRefiner2D, InternalCircularConstraints_CurvedConstraintSplitProducesValidMesh)
{
    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topologyEdges;

    std::vector<std::string> outerEdgeLoop;
    addSquare(*geometry, topologyCorners, topologyEdges, outerEdgeLoop, "outer", 10.0);

    addCircularEdge(*geometry, topologyCorners, topologyEdges, "circle1", 4.0, 5.0, 0.95, 8);
    addCircularEdge(*geometry, topologyCorners, topologyEdges, "circle2", 6.0, 5.0, 1.0, 8);

    // Circles are internal constraints, not holes — empty hole loop list
    auto topology = std::make_unique<Topology2D::Topology2D>(
        topologyCorners, topologyEdges, outerEdgeLoop,
        std::vector<std::vector<std::string>>{});

    MeshingContext2D context(std::move(geometry), std::move(topology));

    BoundaryDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D::triangulate(context, discretization);

    // Enable per-iteration mesh checking so that overlapping triangles (which
    // would otherwise cause an infinite refinement loop) are caught immediately
    // and surface as an exception rather than hanging the test.
    ScopedEnvironmentOverride checkMeshEachIteration("CHECK_MESH_EACH_ITERATION", "1");

    ShewchukRefiner2D refiner(context, Meshing::MeshQualitySettings2D{});
    ASSERT_NO_THROW(refiner.refine());
}

// Regression test: refinement near internal circular constraints must terminate.
// Previously, circumcenters of bad triangles near curved constraints would land
// on the constraint boundary, encroaching ever-smaller segments in an infinite
// cascade of splits.
TEST(ShewchukRefiner2D, SquareWithInternalCirclesTerminates)
{
    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topologyEdges;

    std::vector<std::string> outerEdgeLoop;
    addSquare(*geometry, topologyCorners, topologyEdges, outerEdgeLoop, "outer", 10.0);

    addCircularEdge(*geometry, topologyCorners, topologyEdges, "circle1", 4.0, 5.0, 0.95, 8);
    addCircularEdge(*geometry, topologyCorners, topologyEdges, "circle2", 6.0, 5.0, 1.0, 8);

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topologyCorners, topologyEdges, outerEdgeLoop,
        std::vector<std::vector<std::string>>{});

    MeshingContext2D context(std::move(geometry), std::move(topology));

    BoundaryDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D::triangulate(context, discretization);

    ShewchukRefiner2D refiner(context, Meshing::MeshQualitySettings2D{});

    // This must complete without throwing or running forever.
    // Before the fix, this would enter an infinite encroachment cascade.
    EXPECT_NO_THROW(refiner.refine());

    // Verify we produced a reasonable mesh (not excessively refined)
    const auto& meshData = context.getMeshData();
    EXPECT_GT(meshData.getElementCount(), 0u);
    EXPECT_LT(meshData.getElementCount(), 5000u);
}

// Integration test: two boundary edges declared as twins must end up with
// identical discretization (same number of nodes, matching y-coordinates) after
// Shewchuk refinement propagates every boundary split to the paired edge.
TEST(ShewchukRefiner2D, TwinEdgesHaveMatchingDiscretization)
{
    // Build a 1×1 square. Edges:
    //   sq_e0: c0(0,0) → c1(1,0)  bottom
    //   sq_e1: c1(1,0) → c2(1,1)  right  (bottom-to-top)
    //   sq_e2: c2(1,1) → c3(0,1)  top
    //   sq_e3: c3(0,1) → c0(0,0)  left   (top-to-bottom)
    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topologyEdges;
    std::vector<std::string> edgeLoop;
    addSquare(*geometry, topologyCorners, topologyEdges, edgeLoop, "sq", 1.0);

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topologyCorners, topologyEdges, edgeLoop,
        std::vector<std::vector<std::string>>{});

    MeshingContext2D context(std::move(geometry), std::move(topology));

    BoundaryDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D::triangulate(context, discretization);

    // Locate the four corner nodes by coordinate
    NodeId corner0Id = NodeId::INVALID;
    NodeId corner1Id = NodeId::INVALID;
    NodeId corner2Id = NodeId::INVALID;
    NodeId corner3Id = NodeId::INVALID;
    for (const auto& [nodeId, node] : context.getMeshData().getNodes())
    {
        const Point2D& point = node->getCoordinates();
        if (std::abs(point.x()) < 1e-10 && std::abs(point.y()) < 1e-10)
            corner0Id = nodeId; // (0,0) bottom-left
        else if (std::abs(point.x() - 1.0) < 1e-10 && std::abs(point.y()) < 1e-10)
            corner1Id = nodeId; // (1,0) bottom-right
        else if (std::abs(point.x() - 1.0) < 1e-10 && std::abs(point.y() - 1.0) < 1e-10)
            corner2Id = nodeId; // (1,1) top-right
        else if (std::abs(point.x()) < 1e-10 && std::abs(point.y() - 1.0) < 1e-10)
            corner3Id = nodeId; // (0,1) top-left
    }
    ASSERT_NE(corner0Id, NodeId::INVALID) << "Corner c0 (0,0) not found";
    ASSERT_NE(corner1Id, NodeId::INVALID) << "Corner c1 (1,0) not found";
    ASSERT_NE(corner2Id, NodeId::INVALID) << "Corner c2 (1,1) not found";
    ASSERT_NE(corner3Id, NodeId::INVALID) << "Corner c3 (0,1) not found";

    // Insert an interior node very close to the left edge to guarantee two things:
    //  1. The left edge is encroached (distance 0.1 from diametral-circle centre
    //     (0, 0.5), radius 0.5 → 0.1 < 0.5 ✓)
    //  2. The triangle touching the left edge has a ~11° angle, which fails the
    //     π/6 quality threshold, so the refiner will actually call refineStep().
    context.getOperations().insertVertexBowyerWatson(Point2D(0.1, 0.5));

    // Register the left/right edge pair as twins.
    // Left edge (sq_e3) goes c3→c0 (top-to-bottom).
    // Right edge (sq_e1) goes c1→c2 (bottom-to-top).
    // Correspondence: c3(y=1)↔c2(y=1)  and  c0(y=0)↔c1(y=0).
    TwinManager2D twinManager;
    twinManager.registerTwin(TwinManager2D::NO_SURFACE, corner3Id, corner0Id, TwinManager2D::NO_SURFACE, corner2Id, corner1Id);

    ShewchukRefiner2D refiner(context, Meshing::MeshQualitySettings2D{});
    refiner.setOnBoundarySplit(BoundarySplitSynchronizer2D(context, twinManager));

    ASSERT_NO_THROW(refiner.refine());

    // Collect nodes on the left edge (x ≈ 0) and right edge (x ≈ 1), sorted by y
    std::vector<double> leftY, rightY;
    for (const auto& [nodeId, node] : context.getMeshData().getNodes())
    {
        const Point2D& point = node->getCoordinates();
        if (std::abs(point.x()) < 1e-10)
            leftY.push_back(point.y());
        else if (std::abs(point.x() - 1.0) < 1e-10)
            rightY.push_back(point.y());
    }
    std::sort(leftY.begin(), leftY.end());
    std::sort(rightY.begin(), rightY.end());

    // At least one split must have occurred on each edge
    EXPECT_GT(leftY.size(), 2u);

    // Both edges must have the same number of nodes
    ASSERT_EQ(leftY.size(), rightY.size());

    // Each corresponding node pair must share the same y-coordinate
    for (size_t i = 0; i < leftY.size(); ++i)
    {
        EXPECT_NEAR(leftY[i], rightY[i], 1e-10)
            << "y-mismatch at index " << i
            << ": left=" << leftY[i] << " right=" << rightY[i];
    }
}

// Regression test (OPE-229): on a boundary curve that bulges outward, the
// midpoint of a segment lies beyond its chord, where the exterior triangles have
// been removed. Bowyer-Watson finds nothing in conflict with it there, so the
// split used to fail -- and before OPE-224 that failure aborted the process.
TEST(ShewchukRefiner2D, OutwardBulgingBoundary_SegmentSplitExtendsMeshAcrossChord)
{
    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topologyEdges;

    const size_t numberOfSegments = 3;
    addCircularEdge(*geometry, topologyCorners, topologyEdges, "disc", 0.0, 0.0, 1.0, numberOfSegments);

    std::vector<std::string> outerEdgeLoop;
    for (size_t i = 0; i < numberOfSegments; ++i)
        outerEdgeLoop.push_back("disc_e" + std::to_string(i));

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topologyCorners, topologyEdges, outerEdgeLoop, std::vector<std::vector<std::string>>{});
    MeshingContext2D context(std::move(geometry), std::move(topology));

    BoundaryDiscretizer2D discretizer(context);
    ConstrainedDelaunay2D::triangulate(context, discretizer.discretize());

    std::vector<CurveSegment> segments;
    for (auto& [segmentId, segment] : context.getMeshData().getCurveSegmentManager().getAllSegments())
        segments.push_back(segment);
    ASSERT_FALSE(segments.empty());

    for (auto& segment : segments)
    {
        auto edge = context.getGeometry().getEdge(segment.edgeId);
        auto newNodeId = context.getOperations().splitConstrainedSegment(segment, *edge);
        ASSERT_TRUE(newNodeId.has_value()) << "segment (" << segment.startNodeId.value() << ", " << segment.endNodeId.value() << ")";
    }

    auto verification = MeshVerifier2D::verify(context.getMeshData());
    EXPECT_TRUE(verification.isValid);
}

// Regression test (OPE-239): a hole's arcs bulge into the domain, so a segment's
// midpoint lies on the mesh side of its chord. Bowyer-Watson's fan used to close
// the cavity with the triangle (chord, midpoint) between the chord and the arc,
// inside the hole, where the refiner saw it as a poor triangle it could not refine.
TEST(ShewchukRefiner2D, InwardBulgingBoundary_SegmentSplitLeavesNoTriangleInHole)
{
    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topologyEdges;

    std::vector<std::string> outerEdgeLoop;
    addSquare(*geometry, topologyCorners, topologyEdges, outerEdgeLoop, "outer", 10.0);

    const size_t numberOfSegments = 3;
    addCircularEdge(*geometry, topologyCorners, topologyEdges, "hole", 5.0, 5.0, 2.0, numberOfSegments);
    std::vector<std::string> holeLoop;
    for (size_t i = 0; i < numberOfSegments; ++i)
        holeLoop.push_back("hole_e" + std::to_string(i));

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topologyCorners, topologyEdges, outerEdgeLoop, std::vector<std::vector<std::string>>{holeLoop});
    MeshingContext2D context(std::move(geometry), std::move(topology));

    BoundaryDiscretizer2D discretizer(context);
    ConstrainedDelaunay2D::triangulate(context, discretizer.discretize());

    std::vector<CurveSegment> holeSegments;
    for (auto& [segmentId, segment] : context.getMeshData().getCurveSegmentManager().getAllSegments())
        if (segment.edgeId.rfind("hole", 0) == 0) holeSegments.push_back(segment);
    ASSERT_FALSE(holeSegments.empty());

    for (auto& segment : holeSegments)
    {
        auto edge = context.getGeometry().getEdge(segment.edgeId);
        ASSERT_TRUE(context.getOperations().splitConstrainedSegment(segment, *edge).has_value());
    }

    auto& meshData = context.getMeshData();
    for (auto& [elementId, element] : meshData.getElements())
    {
        auto& nodeIds = dynamic_cast<const TriangleElement&>(*element).getNodeIdArray();
        Point2D centroid = (meshData.getNode(nodeIds[0])->getCoordinates() + meshData.getNode(nodeIds[1])->getCoordinates() +
                            meshData.getNode(nodeIds[2])->getCoordinates()) /
                           3.0;
        EXPECT_GT((centroid - Point2D(5.0, 5.0)).norm(), 2.0) << "triangle " << elementId.value() << " lies in the hole";
    }
    EXPECT_TRUE(MeshVerifier2D::verify(meshData).isValid);
}
