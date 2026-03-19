#include <gtest/gtest.h>

#include "Common/TwinManager.h"
#include "Common/Types.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"
#include "Meshing/Core/2D/BoundarySplitSynchronizer.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/EdgeDiscretizer2D.h"
#include "Meshing/Core/2D/MeshOperations2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/Mesh2DQualitySettings.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Topology2D/Topology2D.h"

#include <Geom2d_Circle.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt2d.hxx>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

namespace
{

void addSquare(Geometry2D::GeometryCollection2D& geometry,
               std::unordered_map<std::string, Topology2D::Corner2D>& topoCorners,
               std::unordered_map<std::string, Topology2D::Edge2D>& topoEdges,
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
        std::string cId = prefix + "_c" + std::to_string(i);
        std::string eId = prefix + "_e" + std::to_string(i);
        std::string prevE = prefix + "_e" + std::to_string((i + 3) % 4);

        geometry.addCorner(std::make_unique<Geometry2D::Corner2D>(cId, corners[i]));
        geometry.addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
            eId, corners[i], corners[(i + 1) % 4]));

        topoCorners.emplace(cId, Topology2D::Corner2D(cId, {eId, prevE}));
        topoEdges.emplace(eId, Topology2D::Edge2D(
                                   eId, cId, prefix + "_c" + std::to_string((i + 1) % 4)));
        edgeLoop.push_back(eId);
    }
}

void addCircularEdge(Geometry2D::GeometryCollection2D& geometry,
                     std::unordered_map<std::string, Topology2D::Corner2D>& topoCorners,
                     std::unordered_map<std::string, Topology2D::Edge2D>& topoEdges,
                     const std::string& prefix,
                     double centerX,
                     double centerY,
                     double radius,
                     size_t numSegments = 8)
{
    gp_Pnt2d center(centerX, centerY);
    gp_Ax2d axis(center, gp_Dir2d(1.0, 0.0));
    gp_Circ2d circle(axis, radius);

    for (size_t i = 0; i < numSegments; ++i)
    {
        double angle = 2.0 * M_PI * i / numSegments;
        double x = centerX + radius * std::cos(angle);
        double y = centerY + radius * std::sin(angle);

        std::string cId = prefix + "_c" + std::to_string(i);
        std::string eId = prefix + "_e" + std::to_string(i);
        std::string prevE = prefix + "_e" + std::to_string((i + numSegments - 1) % numSegments);

        geometry.addCorner(std::make_unique<Geometry2D::OpenCascade2DCorner>(gp_Pnt2d(x, y), cId));
        topoCorners.emplace(cId, Topology2D::Corner2D(cId, {eId, prevE}));
    }

    for (size_t i = 0; i < numSegments; ++i)
    {
        double startAngle = 2.0 * M_PI * i / numSegments;
        double endAngle = 2.0 * M_PI * (i + 1) / numSegments;

        Handle(Geom2d_Circle) circleGeom = new Geom2d_Circle(circle);
        Handle(Geom2d_TrimmedCurve) arc = new Geom2d_TrimmedCurve(circleGeom, startAngle, endAngle);

        std::string eId = prefix + "_e" + std::to_string(i);
        std::string startC = prefix + "_c" + std::to_string(i);
        std::string endC = prefix + "_c" + std::to_string((i + 1) % numSegments);

        geometry.addEdge(std::make_unique<Geometry2D::OpenCascade2DEdge>(arc, eId));
        topoEdges.emplace(eId, Topology2D::Edge2D(eId, startC, endC));
    }
}

} // namespace

// Regression test: refinement near internal circular constraints must terminate.
// Previously, circumcenters of bad triangles near curved constraints would land
// on the constraint boundary, encroaching ever-smaller segments in an infinite
// cascade of splits.
TEST(ShewchukRefiner2D, SquareWithInternalCirclesTerminates)
{
    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;

    std::vector<std::string> outerEdgeLoop;
    addSquare(*geometry, topoCorners, topoEdges, outerEdgeLoop, "outer", 10.0);

    addCircularEdge(*geometry, topoCorners, topoEdges, "circle1", 4.0, 5.0, 0.95, 8);
    addCircularEdge(*geometry, topoCorners, topoEdges, "circle2", 6.0, 5.0, 1.0, 8);

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges, outerEdgeLoop,
        std::vector<std::vector<std::string>>{});

    MeshingContext2D context(std::move(geometry), std::move(topology));

    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D mesher(context, discretization);
    mesher.triangulate();

    ShewchukRefiner2D refiner(context, Meshing::Mesh2DQualitySettings{});

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
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;
    std::vector<std::string> edgeLoop;
    addSquare(*geometry, topoCorners, topoEdges, edgeLoop, "sq", 1.0);

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges, edgeLoop,
        std::vector<std::vector<std::string>>{});

    MeshingContext2D context(std::move(geometry), std::move(topology));

    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D mesher(context, discretization);
    mesher.triangulate();

    // Locate the four corner nodes by coordinate
    size_t c0Id = SIZE_MAX, c1Id = SIZE_MAX, c2Id = SIZE_MAX, c3Id = SIZE_MAX;
    for (const auto& [nodeId, node] : context.getMeshData().getNodes())
    {
        const Point2D& pt = node->getCoordinates();
        if (std::abs(pt.x()) < 1e-10 && std::abs(pt.y()) < 1e-10)
            c0Id = nodeId; // (0,0) bottom-left
        else if (std::abs(pt.x() - 1.0) < 1e-10 && std::abs(pt.y()) < 1e-10)
            c1Id = nodeId; // (1,0) bottom-right
        else if (std::abs(pt.x() - 1.0) < 1e-10 && std::abs(pt.y() - 1.0) < 1e-10)
            c2Id = nodeId; // (1,1) top-right
        else if (std::abs(pt.x()) < 1e-10 && std::abs(pt.y() - 1.0) < 1e-10)
            c3Id = nodeId; // (0,1) top-left
    }
    ASSERT_NE(c0Id, SIZE_MAX) << "Corner c0 (0,0) not found";
    ASSERT_NE(c1Id, SIZE_MAX) << "Corner c1 (1,0) not found";
    ASSERT_NE(c2Id, SIZE_MAX) << "Corner c2 (1,1) not found";
    ASSERT_NE(c3Id, SIZE_MAX) << "Corner c3 (0,1) not found";

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
    TwinManager twinManager;
    twinManager.registerTwin(TwinManager::NO_SURFACE, c3Id, c0Id, TwinManager::NO_SURFACE, c2Id, c1Id);

    ShewchukRefiner2D refiner(context, Meshing::Mesh2DQualitySettings{});
    refiner.setOnBoundarySplit(BoundarySplitSynchronizer(context, twinManager));

    ASSERT_NO_THROW(refiner.refine());

    // Collect nodes on the left edge (x ≈ 0) and right edge (x ≈ 1), sorted by y
    std::vector<double> leftY, rightY;
    for (const auto& [nodeId, node] : context.getMeshData().getNodes())
    {
        const Point2D& pt = node->getCoordinates();
        if (std::abs(pt.x()) < 1e-10)
            leftY.push_back(pt.y());
        else if (std::abs(pt.x() - 1.0) < 1e-10)
            rightY.push_back(pt.y());
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
