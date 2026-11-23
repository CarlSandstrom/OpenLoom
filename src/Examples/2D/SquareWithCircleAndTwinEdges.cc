#include "ExampleShapes2D.h"
#include "Common/Logging.h"
#include "Meshing/Core/2D/Periodic/TwinManager2D.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Meshing/Core/2D/General/BoundaryDiscretizer2D.h"
#include "Meshing/Core/2D/Periodic/BoundarySplitSynchronizer2D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/General/MeshingContext2D.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/MeshQualitySettings2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"


#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

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
    // TwinManager2D callback, keeping both edges identically discretized.
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
    auto outerLoop = ExampleShapes2D::addSquare(*geometry, topologyCorners, topologyEdges, "sq", 10.0);

    // Circular hole — 12 arc segments, center at (5, 8), radius 1.5
    auto holeLoop = ExampleShapes2D::addCircle(*geometry, topologyCorners, topologyEdges, "circ", Point2D(7.0, 8.3), 1.5, 12);

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
    NodeId corner0Id = NodeId::INVALID;
    NodeId corner1Id = NodeId::INVALID;
    NodeId corner2Id = NodeId::INVALID;
    NodeId corner3Id = NodeId::INVALID;
    for (const auto& [nodeId, node] : context.getMeshData().getNodes())
    {
        const Point2D& point = node->getCoordinates();
        if      (std::abs(point.x())        < 1e-9 && std::abs(point.y())        < 1e-9) corner0Id = nodeId;
        else if (std::abs(point.x() - 10.0) < 1e-9 && std::abs(point.y())        < 1e-9) corner1Id = nodeId;
        else if (std::abs(point.x() - 10.0) < 1e-9 && std::abs(point.y() - 10.0) < 1e-9) corner2Id = nodeId;
        else if (std::abs(point.x())        < 1e-9 && std::abs(point.y() - 10.0) < 1e-9) corner3Id = nodeId;
    }
    if (corner0Id == NodeId::INVALID || corner1Id == NodeId::INVALID || corner2Id == NodeId::INVALID || corner3Id == NodeId::INVALID)
    {
        spdlog::error("Could not locate all four corner nodes — aborting");
        return 1;
    }
    spdlog::info("Corner nodes: c0={} (0,0)  c1={} (10,0)  c2={} (10,10)  c3={} (0,10)",
                 corner0Id.value(), corner1Id.value(), corner2Id.value(), corner3Id.value());

    // -------------------------------------------------------------------------
    // TwinManager2D: top edge (e2: c2→c3) ↔ bottom edge (e0: c0→c1)
    //
    //   c2(x=10) corresponds to c1(x=10)  — both at the right end
    //   c3(x=0)  corresponds to c0(x=0)   — both at the left end
    //
    //   registerTwin(c2, c3, c1, c0)  →  getTwin(c2,c3) = (c1,c0)
    //                                     getTwin(c0,c1) = (c3,c2)
    // -------------------------------------------------------------------------
    TwinManager2D twinManager;
    twinManager.registerTwin(TwinManager2D::NO_SURFACE, corner2Id, corner3Id, TwinManager2D::NO_SURFACE, corner1Id, corner0Id);
    spdlog::info("Registered twin pair: top edge (c2→c3) ↔ bottom edge (c1→c0)");

    // -------------------------------------------------------------------------
    // Refine with BoundarySplitSynchronizer2D
    // -------------------------------------------------------------------------
    ShewchukRefiner2D refiner(context, Meshing::MeshQualitySettings2D{});
    refiner.setOnBoundarySplit(BoundarySplitSynchronizer2D(context, twinManager));

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
    Export::TsvExporter::writeMesh(context.getMeshData(), "SquareWithCircleAndTwinEdges");
    spdlog::info("Mesh exported to SquareWithCircleAndTwinEdges.vtu");

    return 0;
}
