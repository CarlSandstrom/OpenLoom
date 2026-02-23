#include "Common/TwinManager.h"
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/EdgeDiscretizer2D.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Core/2D/MeshOperations2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/Shewchuk2DQualityController.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
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
        std::string cId  = prefix + "_c" + std::to_string(i);
        std::string eId  = prefix + "_e" + std::to_string(i);
        std::string prevE = prefix + "_e" + std::to_string((i + 3) % 4);

        geometry.addCorner(std::make_unique<Geometry2D::Corner2D>(cId, corners[i]));
        geometry.addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
            eId, corners[i], corners[(i + 1) % 4]));

        topoCorners.emplace(cId, Topology2D::Corner2D(cId, {eId, prevE}));
        topoEdges.emplace(eId,
            Topology2D::Edge2D(eId, cId, prefix + "_c" + std::to_string((i + 1) % 4)));
        edgeLoop.push_back(eId);
    }
}

static void addCircularHole(Geometry2D::GeometryCollection2D& geometry,
                             std::unordered_map<std::string, Topology2D::Corner2D>& topoCorners,
                             std::unordered_map<std::string, Topology2D::Edge2D>& topoEdges,
                             std::vector<std::string>& edgeLoop,
                             const std::string& prefix,
                             double centerX,
                             double centerY,
                             double radius,
                             size_t numSegments = 12)
{
    gp_Pnt2d center(centerX, centerY);
    gp_Ax2d  axis(center, gp_Dir2d(1.0, 0.0));
    gp_Circ2d circle(axis, radius);

    for (size_t i = 0; i < numSegments; ++i)
    {
        double angle = 2.0 * M_PI * i / numSegments;
        double x     = centerX + radius * std::cos(angle);
        double y     = centerY + radius * std::sin(angle);

        std::string cId   = prefix + "_c" + std::to_string(i);
        std::string eId   = prefix + "_e" + std::to_string(i);
        std::string prevE = prefix + "_e" + std::to_string((i + numSegments - 1) % numSegments);

        geometry.addCorner(
            std::make_unique<Geometry2D::OpenCascade2DCorner>(gp_Pnt2d(x, y), cId));
        topoCorners.emplace(cId, Topology2D::Corner2D(cId, {eId, prevE}));
        edgeLoop.push_back(eId);
    }

    for (size_t i = 0; i < numSegments; ++i)
    {
        double startAngle = 2.0 * M_PI * i / numSegments;
        double endAngle   = 2.0 * M_PI * (i + 1) / numSegments;

        Handle(Geom2d_Circle)      circleGeom = new Geom2d_Circle(circle);
        Handle(Geom2d_TrimmedCurve) arc = new Geom2d_TrimmedCurve(circleGeom, startAngle, endAngle);

        std::string eId    = prefix + "_e" + std::to_string(i);
        std::string startC = prefix + "_c" + std::to_string(i);
        std::string endC   = prefix + "_c" + std::to_string((i + 1) % numSegments);

        geometry.addEdge(std::make_unique<Geometry2D::OpenCascade2DEdge>(arc, eId));
        topoEdges.emplace(eId, Topology2D::Edge2D(eId, startC, endC));
    }
}

int main()
{
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] %^%v%$");

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
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    std::unordered_map<std::string, Topology2D::Edge2D>   topoEdges;

    // Outer square — addSquare produces:
    //   e0: c0(0,0)  → c1(10,0)   bottom  (left-to-right)
    //   e1: c1(10,0) → c2(10,10)  right   (bottom-to-top)
    //   e2: c2(10,10)→ c3(0,10)   top     (right-to-left)
    //   e3: c3(0,10) → c0(0,0)    left    (top-to-bottom)
    std::vector<std::string> outerLoop;
    addSquare(*geometry, topoCorners, topoEdges, outerLoop, "sq", 10.0);

    // Circular hole — 12 arc segments, center at (5, 8), radius 1.5
    std::vector<std::string> holeLoop;
    addCircularHole(*geometry, topoCorners, topoEdges, holeLoop, "circ", 7.0, 8.3, 1.5, 12);

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges, outerLoop,
        std::vector<std::vector<std::string>>{holeLoop});

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
    // Node IDs are assigned by the Delaunay inserter and are not guessable
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
    // TwinManager: top edge (e2: c2→c3) ↔ bottom edge (e0: c0→c1)
    //
    //   c2(x=10) corresponds to c1(x=10)  — both at the right end
    //   c3(x=0)  corresponds to c0(x=0)   — both at the left end
    //
    //   registerTwin(c2, c3, c1, c0)  →  getTwin(c2,c3) = (c1,c0)
    //                                     getTwin(c0,c1) = (c3,c2)
    // -------------------------------------------------------------------------
    TwinManager twinManager;
    twinManager.registerTwin(c2Id, c3Id, c1Id, c0Id);
    spdlog::info("Registered twin pair: top edge (c2→c3) ↔ bottom edge (c1→c0)");

    // -------------------------------------------------------------------------
    // Refine with BoundarySplitCallback
    // -------------------------------------------------------------------------
    Shewchuk2DQualityController qualityController(
        context.getMeshData(),
        2.0,        // max circumradius / shortest-edge ratio
        M_PI / 6.0, // min angle 30°
        10000);     // element limit

    ShewchukRefiner2D refiner(context, qualityController);

    refiner.setOnBoundarySplit([&](size_t n1, size_t n2, size_t mid) {
        auto twin = twinManager.getTwin(n1, n2);
        if (!twin)
            return;
        auto [t1, t2] = *twin;

        // Find the stored segment for the twin (direction may differ from TwinManager)
        const auto& segs = context.getMeshData().getConstrainedSegments();
        auto it = std::find_if(segs.begin(), segs.end(),
                               [&](const ConstrainedSegment2D& s) {
                                   return (s.nodeId1 == t1 && s.nodeId2 == t2) ||
                                          (s.nodeId1 == t2 && s.nodeId2 == t1);
                               });
        if (it == segs.end())
            return;
        const ConstrainedSegment2D twinSeg = *it;

        auto twinEdgeId = context.getOperations().getQueries().findCommonGeometryId(
            twinSeg.nodeId1, twinSeg.nodeId2);
        if (!twinEdgeId)
            return;
        const auto* twinEdge = context.getGeometry().getEdge(*twinEdgeId);
        if (!twinEdge)
            return;

        auto twinMid = context.getOperations().splitConstrainedSegment(twinSeg, *twinEdge);
        if (!twinMid)
            return;

        twinManager.recordSplit(n1, n2, mid, t1, t2, *twinMid);
        spdlog::debug("Twin split: ({},{}) → mid {} / ({},{}) → twinMid {}",
                      n1, n2, mid, t1, t2, *twinMid);
    });

    spdlog::info("Refining...");
    refiner.refine();

    spdlog::info("Refinement complete: {} triangles", context.getMeshData().getElementCount());

    // -------------------------------------------------------------------------
    // Report: collect top-edge (y≈10) and bottom-edge (y≈0) nodes, sorted by x
    // -------------------------------------------------------------------------
    std::vector<double> topX, bottomX;
    for (const auto& [nodeId, node] : context.getMeshData().getNodes())
    {
        const Point2D& pt = node->getCoordinates();
        if (std::abs(pt.y() - 10.0) < 1e-9)
            topX.push_back(pt.x());
        else if (std::abs(pt.y()) < 1e-9)
            bottomX.push_back(pt.x());
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
