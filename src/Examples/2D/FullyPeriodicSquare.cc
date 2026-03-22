#include "Common/TwinManager.h"
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"
#include "Meshing/Core/2D/BoundarySplitSynchronizer.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/EdgeDiscretizer2D.h"
#include "Meshing/Core/2D/Mesh2DQualitySettings.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Topology2D/Topology2D.h"
#include "Common/Logging.h"
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
        std::string cornerId   = prefix + "_c" + std::to_string(i);
        std::string edgeId     = prefix + "_e" + std::to_string(i);
        std::string prevEdgeId = prefix + "_e" + std::to_string((i + 3) % 4);

        geometry.addCorner(std::make_unique<Geometry2D::Corner2D>(cornerId, corners[i]));
        geometry.addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
            edgeId, corners[i], corners[(i + 1) % 4]));

        topoCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, prevEdgeId}));
        topoEdges.emplace(edgeId, Topology2D::Edge2D(
            edgeId, cornerId, prefix + "_c" + std::to_string((i + 1) % 4)));
        edgeLoop.push_back(edgeId);
    }
}

static void addCircularConstraint(Geometry2D::GeometryCollection2D& geometry,
                                  std::unordered_map<std::string, Topology2D::Corner2D>& topoCorners,
                                  std::unordered_map<std::string, Topology2D::Edge2D>& topoEdges,
                                  const std::string& prefix,
                                  double centerX,
                                  double centerY,
                                  double radius,
                                  size_t numSegments)
{
    gp_Pnt2d center(centerX, centerY);
    gp_Ax2d  axis(center, gp_Dir2d(1.0, 0.0));
    gp_Circ2d circle(axis, radius);

    for (size_t i = 0; i < numSegments; ++i)
    {
        double angle = 2.0 * M_PI * i / numSegments;
        double x     = centerX + radius * std::cos(angle);
        double y     = centerY + radius * std::sin(angle);

        std::string cornerId   = prefix + "_c" + std::to_string(i);
        std::string edgeId     = prefix + "_e" + std::to_string(i);
        std::string prevEdgeId = prefix + "_e" + std::to_string((i + numSegments - 1) % numSegments);

        geometry.addCorner(
            std::make_unique<Geometry2D::OpenCascade2DCorner>(gp_Pnt2d(x, y), cornerId));
        topoCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, prevEdgeId}));
    }

    for (size_t i = 0; i < numSegments; ++i)
    {
        double startAngle = 2.0 * M_PI * i / numSegments;
        double endAngle   = 2.0 * M_PI * (i + 1) / numSegments;

        Handle(Geom2d_Circle)       circleGeom = new Geom2d_Circle(circle);
        Handle(Geom2d_TrimmedCurve) arc        = new Geom2d_TrimmedCurve(circleGeom, startAngle, endAngle);

        std::string edgeId        = prefix + "_e" + std::to_string(i);
        std::string startCornerId = prefix + "_c" + std::to_string(i);
        std::string endCornerId   = prefix + "_c" + std::to_string((i + 1) % numSegments);

        geometry.addEdge(std::make_unique<Geometry2D::OpenCascade2DEdge>(arc, edgeId));
        topoEdges.emplace(edgeId, Topology2D::Edge2D(edgeId, startCornerId, endCornerId));
    }
}

int main()
{
    Common::initLogging();

    // -------------------------------------------------------------------------
    // Geometry: 10×10 square with two circular holes and all four boundary
    // edges twinned in two pairs:
    //
    //   y=10  ---- top (sq_e2) ↔ bottom (sq_e0) ----
    //         |                                     |
    //         |  [circ_left]          [circ_top]    |
    //         |  center(1.5, 5)       center(5, 8.5)|
    //         |  r=1.0, 12 seg        r=1.0, 12 seg |
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
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    std::unordered_map<std::string, Topology2D::Edge2D>   topoEdges;

    // Outer square: sq_e0=bottom, sq_e1=right, sq_e2=top, sq_e3=left
    //   sq_c0=(0,0)  sq_c1=(10,0)  sq_c2=(10,10)  sq_c3=(0,10)
    std::vector<std::string> outerLoop;
    addSquare(*geometry, topoCorners, topoEdges, outerLoop, "sq", 10.0);

    // Circular hole near the left edge — forces left/right twin pair splits
    const size_t numSegments = 12;
    addCircularConstraint(*geometry, topoCorners, topoEdges, "circ_left", 1.5, 5.0, 1.0, numSegments);
    std::vector<std::string> holeLeftLoop;
    for (size_t i = 0; i < numSegments; ++i)
        holeLeftLoop.push_back("circ_left_e" + std::to_string(i));

    // Circular hole near the top edge — forces top/bottom twin pair splits
    addCircularConstraint(*geometry, topoCorners, topoEdges, "circ_top", 5.0, 8.5, 1.0, numSegments);
    std::vector<std::string> holeTopLoop;
    for (size_t i = 0; i < numSegments; ++i)
        holeTopLoop.push_back("circ_top_e" + std::to_string(i));

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges, outerLoop,
        std::vector<std::vector<std::string>>{holeLeftLoop, holeTopLoop});

    MeshingContext2D context(std::move(geometry), std::move(topology));

    // -------------------------------------------------------------------------
    // Discretize and triangulate
    // -------------------------------------------------------------------------
    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    spdlog::info("Triangulating...");
    ConstrainedDelaunay2D mesher(context, discretization);
    mesher.triangulate();

    spdlog::info("Triangulation complete: {} triangles", context.getMeshData().getElementCount());

    // -------------------------------------------------------------------------
    // Locate the four corner nodes by coordinate after triangulation.
    // Node IDs are assigned by the Delaunay inserter and are not predictable
    // ahead of time, so we scan the mesh.
    // -------------------------------------------------------------------------
    size_t c0Id = SIZE_MAX, c1Id = SIZE_MAX, c2Id = SIZE_MAX, c3Id = SIZE_MAX;
    for (const auto& [nodeId, node] : context.getMeshData().getNodes())
    {
        const Point2D& pt = node->getCoordinates();
        if      (std::abs(pt.x())        < 1e-9 && std::abs(pt.y())        < 1e-9) c0Id = nodeId;
        else if (std::abs(pt.x() - 10.0) < 1e-9 && std::abs(pt.y())        < 1e-9) c1Id = nodeId;
        else if (std::abs(pt.x() - 10.0) < 1e-9 && std::abs(pt.y() - 10.0) < 1e-9) c2Id = nodeId;
        else if (std::abs(pt.x())        < 1e-9 && std::abs(pt.y() - 10.0) < 1e-9) c3Id = nodeId;
    }
    if (c0Id == SIZE_MAX || c1Id == SIZE_MAX || c2Id == SIZE_MAX || c3Id == SIZE_MAX)
    {
        spdlog::error("Could not locate all four corner nodes — aborting");
        return 1;
    }
    spdlog::info("Corner nodes: c0={} (0,0)  c1={} (10,0)  c2={} (10,10)  c3={} (0,10)",
                 c0Id, c1Id, c2Id, c3Id);

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
    twinManager.registerTwin(TwinManager::NO_SURFACE, c2Id, c3Id, TwinManager::NO_SURFACE, c1Id, c0Id);
    spdlog::info("Registered twin pair: top edge (c2→c3) ↔ bottom edge (c1→c0)");

    twinManager.registerTwin(TwinManager::NO_SURFACE, c3Id, c0Id, TwinManager::NO_SURFACE, c2Id, c1Id);
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
        const Point2D& pt = node->getCoordinates();
        if      (std::abs(pt.y() - 10.0) < 1e-9) topX.push_back(pt.x());
        else if (std::abs(pt.y())         < 1e-9) bottomX.push_back(pt.x());
        if      (std::abs(pt.x())         < 1e-9) leftY.push_back(pt.y());
        else if (std::abs(pt.x() - 10.0)  < 1e-9) rightY.push_back(pt.y());
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
