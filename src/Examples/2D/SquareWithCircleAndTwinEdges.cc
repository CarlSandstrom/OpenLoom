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
#include "Meshing/Data/2D/Node2D.h"
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
        topologyEdges.emplace(edgeId,
            Topology2D::Edge2D(edgeId, cornerId, prefix + "_c" + std::to_string((i + 1) % 4)));
        edgeLoop.push_back(edgeId);
    }
}

static void addCircularHole(Geometry2D::GeometryCollection2D& geometry,
                             std::unordered_map<std::string, Topology2D::Corner2D>& topologyCorners,
                             std::unordered_map<std::string, Topology2D::Edge2D>& topologyEdges,
                             std::vector<std::string>& edgeLoop,
                             const std::string& prefix,
                             double centerX,
                             double centerY,
                             double radius,
                             size_t numberOfSegments = 12)
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
        edgeLoop.push_back(edgeId);
    }

    for (size_t i = 0; i < numberOfSegments; ++i)
    {
        double startAngle = 2.0 * M_PI * i / numberOfSegments;
        double endAngle   = 2.0 * M_PI * (i + 1) / numberOfSegments;

        Handle(Geom2d_Circle)      circleGeometry = new Geom2d_Circle(circle);
        Handle(Geom2d_TrimmedCurve) arc = new Geom2d_TrimmedCurve(circleGeometry, startAngle, endAngle);

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
    // Geometry: 10×10 square with a circular hole near the top edge.
    //
    //   y=10  ---- top edge (e2, twin of bottom) ----
    //         |                                     |
    //         |           [ circle ]                |
    //         |       center (5,8), r=1.5           |
    //         |                                     |
    //   y=0   ---- bottom edge (e0, twin of top) ---
    //         x=0                               x=10
    //
    // The circle (clearance 0.5 from top) forces the refiner to split the top
    // edge. Every top-edge split is propagated to the bottom edge by the
    // TwinManager callback, keeping both edges identically discretized.
    // -------------------------------------------------------------------------

    spdlog::info("Building square with circular hole near top edge");

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    std::unordered_map<std::string, Topology2D::Edge2D>   topologyEdges;

    // Outer square — addSquare produces:
    //   e0: c0(0,0)  → c1(10,0)   bottom  (left-to-right)
    //   e1: c1(10,0) → c2(10,10)  right   (bottom-to-top)
    //   e2: c2(10,10)→ c3(0,10)   top     (right-to-left)
    //   e3: c3(0,10) → c0(0,0)    left    (top-to-bottom)
    std::vector<std::string> outerLoop;
    addSquare(*geometry, topologyCorners, topologyEdges, outerLoop, "sq", 10.0);

    // Circular hole — 12 arc segments, center at (5, 8), radius 1.5
    std::vector<std::string> holeLoop;
    addCircularHole(*geometry, topologyCorners, topologyEdges, holeLoop, "circ", 7.0, 8.3, 1.5, 12);

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topologyCorners, topologyEdges, outerLoop,
        std::vector<std::vector<std::string>>{holeLoop});

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
    // Node IDs are assigned by the Delaunay inserter and are not guessable
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
    // TwinManager: top edge (e2: c2→c3) ↔ bottom edge (e0: c0→c1)
    //
    //   c2(x=10) corresponds to c1(x=10)  — both at the right end
    //   c3(x=0)  corresponds to c0(x=0)   — both at the left end
    //
    //   registerTwin(c2, c3, c1, c0)  →  getTwin(c2,c3) = (c1,c0)
    //                                     getTwin(c0,c1) = (c3,c2)
    // -------------------------------------------------------------------------
    TwinManager twinManager;
    twinManager.registerTwin(TwinManager::NO_SURFACE, corner2Id, corner3Id, TwinManager::NO_SURFACE, corner1Id, corner0Id);
    spdlog::info("Registered twin pair: top edge (c2→c3) ↔ bottom edge (c1→c0)");

    // -------------------------------------------------------------------------
    // Refine with BoundarySplitSynchronizer
    // -------------------------------------------------------------------------
    ShewchukRefiner2D refiner(context, Meshing::Mesh2DQualitySettings{});
    refiner.setOnBoundarySplit(BoundarySplitSynchronizer(context, twinManager));

    spdlog::info("Refining...");
    refiner.refine();

    spdlog::info("Refinement complete: {} triangles", context.getMeshData().getElementCount());

    // -------------------------------------------------------------------------
    // Report: collect top-edge (y≈10) and bottom-edge (y≈0) nodes, sorted by x
    // -------------------------------------------------------------------------
    std::vector<double> topX, bottomX;
    for (const auto& [nodeId, node] : context.getMeshData().getNodes())
    {
        const Point2D& point = node->getCoordinates();
        if (std::abs(point.y() - 10.0) < 1e-9)
            topX.push_back(point.x());
        else if (std::abs(point.y()) < 1e-9)
            bottomX.push_back(point.x());
    }
    std::sort(topX.begin(),    topX.end());
    std::sort(bottomX.begin(), bottomX.end());

    spdlog::info("Top edge:    {} nodes at x = {}", topX.size(),
                 [&] { std::string s; for (double v : topX) s += " " + std::to_string(v); return s; }());
    spdlog::info("Bottom edge: {} nodes at x = {}", bottomX.size(),
                 [&] { std::string s; for (double v : bottomX) s += " " + std::to_string(v); return s; }());

    bool match = (topX.size() == bottomX.size());
    if (match)
    {
        for (size_t i = 0; i < topX.size() && match; ++i)
            match = (std::abs(topX[i] - bottomX[i]) < 1e-9);
    }
    spdlog::info("Twin discretization check: {}", match ? "PASS — edges match" : "FAIL — edges differ");

    // -------------------------------------------------------------------------
    // Export
    // -------------------------------------------------------------------------
    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "SquareWithCircleAndTwinEdges.vtu");
    spdlog::info("Mesh exported to SquareWithCircleAndTwinEdges.vtu");

    return 0;
}
