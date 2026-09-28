#include "Common/Logging.h"
#include "Common/TwinManager.h"
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"
#include "Meshing/Core/2D/BoundaryDiscretizer2D.h"
#include "Meshing/Core/2D/BoundarySplitSynchronizer.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/Mesh2DQualitySettings.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"

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

static void addSquare(Geometry2D::GeometryCollection2D& geometry,
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
        std::string cornerId       = prefix + "_c" + std::to_string(i);
        std::string edgeId         = prefix + "_e" + std::to_string(i);
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

static void addCircularConstraint(Geometry2D::GeometryCollection2D& geometry,
                                  std::unordered_map<std::string, Topology2D::Corner2D>& topologyCorners,
                                  std::unordered_map<std::string, Topology2D::Edge2D>& topologyEdges,
                                  const std::string& prefix,
                                  double centerX,
                                  double centerY,
                                  double radius,
                                  size_t numberOfSegments)
{
    gp_Pnt2d center(centerX, centerY);
    gp_Ax2d  axis(center, gp_Dir2d(1.0, 0.0));
    gp_Circ2d circle(axis, radius);

    for (size_t i = 0; i < numberOfSegments; ++i)
    {
        double angle = 2.0 * M_PI * i / numberOfSegments;
        double x     = centerX + radius * std::cos(angle);
        double y     = centerY + radius * std::sin(angle);

        std::string cornerId       = prefix + "_c" + std::to_string(i);
        std::string edgeId         = prefix + "_e" + std::to_string(i);
        std::string previousEdgeId = prefix + "_e" + std::to_string((i + numberOfSegments - 1) % numberOfSegments);

        geometry.addCorner(
            std::make_unique<Geometry2D::OpenCascade2DCorner>(gp_Pnt2d(x, y), cornerId));
        topologyCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, previousEdgeId}));
    }

    for (size_t i = 0; i < numberOfSegments; ++i)
    {
        double startAngle = 2.0 * M_PI * i / numberOfSegments;
        double endAngle   = 2.0 * M_PI * (i + 1) / numberOfSegments;

        Handle(Geom2d_Circle)       circleGeometry = new Geom2d_Circle(circle);
        Handle(Geom2d_TrimmedCurve) arc            = new Geom2d_TrimmedCurve(circleGeometry, startAngle, endAngle);

        std::string edgeId        = prefix + "_e" + std::to_string(i);
        std::string startCornerId = prefix + "_c" + std::to_string(i);
        std::string endCornerId   = prefix + "_c" + std::to_string((i + 1) % numberOfSegments);

        geometry.addEdge(std::make_unique<Geometry2D::OpenCascade2DEdge>(arc, edgeId));
        topologyEdges.emplace(edgeId, Topology2D::Edge2D(edgeId, startCornerId, endCornerId));
    }
}

int main()
{
    Common::initializeLogging();

    // -------------------------------------------------------------------------
    // Geometry: 10×10 square with two circular holes and all four boundary
    // edges twinned in two pairs:
    //
    //   y=10  ---- top (sq_e2) ↔ bottom (sq_e0) ----
    //         |                                     |
    //         |  [circ_left]          [circ_top]    |
    //         |  center(1.5, 5)       center(5, 8.5)|
    //         |  r=1.0, 12 segment        r=1.0, 12 segment |
    //         |                                     |
    //   y=0   ---- bottom (sq_e0) ↔ top (sq_e2) ---
    //         x=0                               x=10
    //
    //   top ↔ bottom  (sq_e2: c2→c3) ↔ (sq_e0: c0→c1): x-coordinate preserved
    //   left ↔ right  (sq_e3: c3→c0) ↔ (sq_e1: c1→c2): y-coordinate preserved
    //
    // circ_left forces splits on the left edge  → propagated to the right edge.
    // circ_top  forces splits on the top edge   → propagated to the bottom edge.
    // Both twin pairs are exercised simultaneously, including cross-pair interaction
    // at the four corners shared by both pairs.
    // -------------------------------------------------------------------------

    spdlog::info("Building fully periodic square with twin edges on all four sides");

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    std::unordered_map<std::string, Topology2D::Edge2D>   topologyEdges;

    // Outer square: sq_e0=bottom, sq_e1=right, sq_e2=top, sq_e3=left
    //   sq_c0=(0,0)  sq_c1=(10,0)  sq_c2=(10,10)  sq_c3=(0,10)
    std::vector<std::string> outerLoop;
    addSquare(*geometry, topologyCorners, topologyEdges, outerLoop, "sq", 10.0);

    // Circular hole near the left edge — forces left/right twin pair splits
    const size_t numberOfSegments = 12;
    addCircularConstraint(*geometry, topologyCorners, topologyEdges, "circ_left", 1.5, 5.0, 1.0, numberOfSegments);
    std::vector<std::string> holeLeftLoop;
    for (size_t i = 0; i < numberOfSegments; ++i)
        holeLeftLoop.push_back("circ_left_e" + std::to_string(i));

    // Circular hole near the top edge — forces top/bottom twin pair splits
    addCircularConstraint(*geometry, topologyCorners, topologyEdges, "circ_top", 5.0, 8.5, 1.0, numberOfSegments);
    std::vector<std::string> holeTopLoop;
    for (size_t i = 0; i < numberOfSegments; ++i)
        holeTopLoop.push_back("circ_top_e" + std::to_string(i));

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topologyCorners, topologyEdges, outerLoop,
        std::vector<std::vector<std::string>>{holeLeftLoop, holeTopLoop});

    MeshingContext2D context(std::move(geometry), std::move(topology));

    // -------------------------------------------------------------------------
    // Discretize and triangulate
    // -------------------------------------------------------------------------
    BoundaryDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    spdlog::info("Triangulating...");
    ConstrainedDelaunay2D::triangulate(context, discretization);

    spdlog::info("Triangulation complete: {} triangles", context.getMeshData().getElementCount());

    // -------------------------------------------------------------------------
    // Locate the four corner nodes by coordinate after triangulation.
    // Node IDs are assigned by the Delaunay inserter and are not predictable
    // ahead of time, so we scan the mesh.
    // -------------------------------------------------------------------------
    size_t corner0Id = SIZE_MAX, corner1Id = SIZE_MAX, corner2Id = SIZE_MAX, corner3Id = SIZE_MAX;
    for (const auto& [nodeId, node] : context.getMeshData().getNodes())
    {
        const Point2D& point = node->getCoordinates();
        if      (std::abs(point.x())        < 1e-9 && std::abs(point.y())        < 1e-9) corner0Id = nodeId;
        else if (std::abs(point.x() - 10.0) < 1e-9 && std::abs(point.y())        < 1e-9) corner1Id = nodeId;
        else if (std::abs(point.x() - 10.0) < 1e-9 && std::abs(point.y() - 10.0) < 1e-9) corner2Id = nodeId;
        else if (std::abs(point.x())        < 1e-9 && std::abs(point.y() - 10.0) < 1e-9) corner3Id = nodeId;
    }
    if (corner0Id == SIZE_MAX || corner1Id == SIZE_MAX || corner2Id == SIZE_MAX || corner3Id == SIZE_MAX)
    {
        spdlog::error("Could not locate all four corner nodes — aborting");
        return 1;
    }
    spdlog::info("Corner nodes: c0={} (0,0)  c1={} (10,0)  c2={} (10,10)  c3={} (0,10)",
                 corner0Id, corner1Id, corner2Id, corner3Id);

    // -------------------------------------------------------------------------
    // TwinManager: register both periodic pairs.
    //
    //   Top ↔ bottom: top edge (c2→c3, right-to-left) ↔ bottom edge (c1→c0)
    //     c2(10,10) ↔ c1(10,0)   both at x=10
    //     c3(0,10)  ↔ c0(0,0)    both at x=0
    //
    //   Left ↔ right: left edge (c3→c0, top-to-bottom) ↔ right edge (c2→c1)
    //     c3(0,10)  ↔ c2(10,10)  both at y=10
    //     c0(0,0)   ↔ c1(10,0)   both at y=0
    // -------------------------------------------------------------------------
    TwinManager twinManager;
    twinManager.registerTwin(TwinManager::NO_SURFACE, corner2Id, corner3Id, TwinManager::NO_SURFACE, corner1Id, corner0Id);
    spdlog::info("Registered twin pair: top edge (c2→c3) ↔ bottom edge (c1→c0)");

    twinManager.registerTwin(TwinManager::NO_SURFACE, corner3Id, corner0Id, TwinManager::NO_SURFACE, corner2Id, corner1Id);
    spdlog::info("Registered twin pair: left edge (c3→c0) ↔ right edge (c2→c1)");

    // -------------------------------------------------------------------------
    // Refine with BoundarySplitSynchronizer
    // -------------------------------------------------------------------------
    ShewchukRefiner2D refiner(context, Meshing::Mesh2DQualitySettings{});
    refiner.setOnBoundarySplit(BoundarySplitSynchronizer(context, twinManager));

    spdlog::info("Refining...");
    refiner.refine();

    spdlog::info("Refinement complete: {} triangles", context.getMeshData().getElementCount());

    // -------------------------------------------------------------------------
    // Validate both twin pairs: collect nodes on each edge, sort by the
    // preserved coordinate, and confirm the two edges in each pair match.
    // -------------------------------------------------------------------------
    std::vector<double> topX, bottomX, leftY, rightY;
    for (const auto& [nodeId, node] : context.getMeshData().getNodes())
    {
        const Point2D& point = node->getCoordinates();
        if      (std::abs(point.y() - 10.0) < 1e-9) topX.push_back(point.x());
        else if (std::abs(point.y())         < 1e-9) bottomX.push_back(point.x());
        if      (std::abs(point.x())         < 1e-9) leftY.push_back(point.y());
        else if (std::abs(point.x() - 10.0)  < 1e-9) rightY.push_back(point.y());
    }
    std::sort(topX.begin(),    topX.end());
    std::sort(bottomX.begin(), bottomX.end());
    std::sort(leftY.begin(),   leftY.end());
    std::sort(rightY.begin(),  rightY.end());

    spdlog::info("Top edge:    {} nodes", topX.size());
    spdlog::info("Bottom edge: {} nodes", bottomX.size());
    bool topBottomMatch = (topX.size() == bottomX.size());
    if (topBottomMatch)
        for (size_t i = 0; i < topX.size() && topBottomMatch; ++i)
            topBottomMatch = (std::abs(topX[i] - bottomX[i]) < 1e-9);
    spdlog::info("Twin check top/bottom: {}", topBottomMatch ? "PASS — edges match" : "FAIL — edges differ");

    spdlog::info("Left edge:   {} nodes", leftY.size());
    spdlog::info("Right edge:  {} nodes", rightY.size());
    bool leftRightMatch = (leftY.size() == rightY.size());
    if (leftRightMatch)
        for (size_t i = 0; i < leftY.size() && leftRightMatch; ++i)
            leftRightMatch = (std::abs(leftY[i] - rightY[i]) < 1e-9);
    spdlog::info("Twin check left/right: {}", leftRightMatch ? "PASS — edges match" : "FAIL — edges differ");

    spdlog::info("Overall: {}", (topBottomMatch && leftRightMatch) ? "PASS" : "FAIL");

    // -------------------------------------------------------------------------
    // Export
    // -------------------------------------------------------------------------
    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "FullyPeriodicSquare.vtu");
    spdlog::info("Mesh exported to FullyPeriodicSquare.vtu");

    return 0;
}
