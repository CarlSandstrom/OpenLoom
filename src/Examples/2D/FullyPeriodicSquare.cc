#include "ExampleShapes2D.h"
#include "Common/Logging.h"
#include "Meshing/Core/2D/Periodic/TwinManager2D.h"
#include "Common/Types.h"
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
#include "Topology2D/Corner2D.h"
#include "Topology2D/Edge2D.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"


#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

// Geometry: 10x10 square with two circular holes and all four boundary
// edges twinned in two pairs:
//
//   y=10  ---- top (sq_e2) <-> bottom (sq_e0) ----
//         |                                       |
//         |  [circ_left]          [circ_top]      |
//         |  center (1.5, 5)      center (5, 8.5) |
//         |  r=1.0, 12 segments   r=1.0, 12 segments
//         |                                       |
//   y=0   ---- bottom (sq_e0) <-> top (sq_e2) ----
//         x=0                                  x=10
//
//   top <-> bottom  (sq_e2: c2->c3) <-> (sq_e0: c0->c1): x-coordinate preserved
//   left <-> right  (sq_e3: c3->c0) <-> (sq_e1: c1->c2): y-coordinate preserved
//
// circ_left forces splits on the left edge, propagated to the right edge.
// circ_top forces splits on the top edge, propagated to the bottom edge.
// Both twin pairs are exercised simultaneously, including cross-pair interaction
// at the four corners shared by both pairs.

namespace
{
constexpr double SQUARE_SIZE = 10.0;
constexpr double HOLE_RADIUS = 1.0;
constexpr std::size_t HOLE_SEGMENT_COUNT = 12;
constexpr double LEFT_HOLE_CENTER_X = 1.5;
constexpr double LEFT_HOLE_CENTER_Y = 5.0;
constexpr double TOP_HOLE_CENTER_X = 5.0;
constexpr double TOP_HOLE_CENTER_Y = 8.5;
constexpr double COORDINATE_TOLERANCE = 1e-9;

using TopologyCorners = std::unordered_map<std::string, Topology2D::Corner2D>;
using TopologyEdges = std::unordered_map<std::string, Topology2D::Edge2D>;

struct SquareCornerNodes
{
    NodeId corner0; // (0, 0)
    NodeId corner1; // (10, 0)
    NodeId corner2; // (10, 10)
    NodeId corner3; // (0, 10)
};

struct BoundaryCoordinates
{
    std::vector<double> topX;
    std::vector<double> bottomX;
    std::vector<double> leftY;
    std::vector<double> rightY;
};

bool isNear(double value, double target)
{
    return std::abs(value - target) < COORDINATE_TOLERANCE;
}

MeshingContext2D buildPeriodicSquareContext()
{
    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    TopologyCorners topologyCorners;
    TopologyEdges topologyEdges;

    auto outerLoop = ExampleShapes2D::addSquare(*geometry, topologyCorners, topologyEdges, "sq", SQUARE_SIZE);
    auto holeLeftLoop = ExampleShapes2D::addCircle(*geometry, topologyCorners, topologyEdges, "circ_left",
                                                   Point2D(LEFT_HOLE_CENTER_X, LEFT_HOLE_CENTER_Y), HOLE_RADIUS, HOLE_SEGMENT_COUNT);
    auto holeTopLoop = ExampleShapes2D::addCircle(*geometry, topologyCorners, topologyEdges, "circ_top",
                                                  Point2D(TOP_HOLE_CENTER_X, TOP_HOLE_CENTER_Y), HOLE_RADIUS, HOLE_SEGMENT_COUNT);

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topologyCorners, topologyEdges, outerLoop, std::vector<std::vector<std::string>>{holeLeftLoop, holeTopLoop});

    return MeshingContext2D(std::move(geometry), std::move(topology));
}

std::optional<NodeId> findNodeAt(const MeshData2D& meshData, const Point2D& target)
{
    std::optional<NodeId> foundNodeId;
    for (auto& [nodeId, node] : meshData.getNodes())
    {
        auto& point = node->getCoordinates();
        if (isNear(point.x(), target.x()) && isNear(point.y(), target.y())) foundNodeId = nodeId;
    }
    return foundNodeId;
}

// Node IDs are assigned by the Delaunay inserter and are not predictable
// ahead of time, so the corners are located by coordinate.
std::optional<SquareCornerNodes> findSquareCornerNodes(const MeshData2D& meshData)
{
    auto corner0 = findNodeAt(meshData, Point2D(0.0, 0.0));
    auto corner1 = findNodeAt(meshData, Point2D(SQUARE_SIZE, 0.0));
    auto corner2 = findNodeAt(meshData, Point2D(SQUARE_SIZE, SQUARE_SIZE));
    auto corner3 = findNodeAt(meshData, Point2D(0.0, SQUARE_SIZE));
    if (!corner0 || !corner1 || !corner2 || !corner3) return std::nullopt;
    return SquareCornerNodes{*corner0, *corner1, *corner2, *corner3};
}

//   Top <-> bottom: top edge (c2->c3, right-to-left) <-> bottom edge (c1->c0)
//     c2(10,10) <-> c1(10,0)   both at x=10
//     c3(0,10)  <-> c0(0,0)    both at x=0
//
//   Left <-> right: left edge (c3->c0, top-to-bottom) <-> right edge (c2->c1)
//     c3(0,10)  <-> c2(10,10)  both at y=10
//     c0(0,0)   <-> c1(10,0)   both at y=0
TwinManager2D registerPeriodicPairs(const SquareCornerNodes& corners)
{
    TwinManager2D twinManager;
    twinManager.registerTwin(TwinManager2D::NO_SURFACE, corners.corner2, corners.corner3, TwinManager2D::NO_SURFACE,
                             corners.corner1, corners.corner0);
    spdlog::info("Registered twin pair: top edge (c2→c3) ↔ bottom edge (c1→c0)");

    twinManager.registerTwin(TwinManager2D::NO_SURFACE, corners.corner3, corners.corner0, TwinManager2D::NO_SURFACE,
                             corners.corner2, corners.corner1);
    spdlog::info("Registered twin pair: left edge (c3→c0) ↔ right edge (c2→c1)");
    return twinManager;
}

// Each list is sorted by the coordinate its twin pair preserves.
BoundaryCoordinates collectBoundaryCoordinates(const MeshData2D& meshData)
{
    BoundaryCoordinates boundary;
    for (auto& [nodeId, node] : meshData.getNodes())
    {
        auto& point = node->getCoordinates();
        if (isNear(point.y(), SQUARE_SIZE))
            boundary.topX.push_back(point.x());
        else if (isNear(point.y(), 0.0))
            boundary.bottomX.push_back(point.x());
        if (isNear(point.x(), 0.0))
            boundary.leftY.push_back(point.y());
        else if (isNear(point.x(), SQUARE_SIZE))
            boundary.rightY.push_back(point.y());
    }
    std::sort(boundary.topX.begin(), boundary.topX.end());
    std::sort(boundary.bottomX.begin(), boundary.bottomX.end());
    std::sort(boundary.leftY.begin(), boundary.leftY.end());
    std::sort(boundary.rightY.begin(), boundary.rightY.end());
    return boundary;
}

bool coordinatesMatch(const std::vector<double>& first, const std::vector<double>& second)
{
    return std::equal(first.begin(), first.end(), second.begin(), second.end(),
                      [](double firstValue, double secondValue)
                      { return isNear(firstValue, secondValue); });
}

bool validateTwinPairs(const MeshData2D& meshData)
{
    auto boundary = collectBoundaryCoordinates(meshData);

    spdlog::info("Top edge:    {} nodes", boundary.topX.size());
    spdlog::info("Bottom edge: {} nodes", boundary.bottomX.size());
    auto topBottomMatch = coordinatesMatch(boundary.topX, boundary.bottomX);
    spdlog::info("Twin check top/bottom: {}", topBottomMatch ? "PASS — edges match" : "FAIL — edges differ");

    spdlog::info("Left edge:   {} nodes", boundary.leftY.size());
    spdlog::info("Right edge:  {} nodes", boundary.rightY.size());
    auto leftRightMatch = coordinatesMatch(boundary.leftY, boundary.rightY);
    spdlog::info("Twin check left/right: {}", leftRightMatch ? "PASS — edges match" : "FAIL — edges differ");

    return topBottomMatch && leftRightMatch;
}
} // namespace

int main()
{
    Common::initializeLogging();

    spdlog::info("Building fully periodic square with twin edges on all four sides");
    auto context = buildPeriodicSquareContext();

    BoundaryDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    spdlog::info("Triangulating...");
    ConstrainedDelaunay2D::triangulate(context, discretization);
    spdlog::info("Triangulation complete: {} triangles", context.getMeshData().getElementCount());

    auto corners = findSquareCornerNodes(context.getMeshData());
    if (!corners)
    {
        spdlog::error("Could not locate all four corner nodes — aborting");
        return 1;
    }
    spdlog::info("Corner nodes: c0={} (0,0)  c1={} (10,0)  c2={} (10,10)  c3={} (0,10)", corners->corner0.value(),
                 corners->corner1.value(), corners->corner2.value(), corners->corner3.value());

    auto twinManager = registerPeriodicPairs(*corners);

    ShewchukRefiner2D refiner(context, MeshQualitySettings2D{});
    refiner.setOnBoundarySplit(BoundarySplitSynchronizer2D(context, twinManager));

    spdlog::info("Refining...");
    refiner.refine();
    spdlog::info("Refinement complete: {} triangles", context.getMeshData().getElementCount());

    auto twinPairsMatch = validateTwinPairs(context.getMeshData());
    spdlog::info("Overall: {}", twinPairsMatch ? "PASS" : "FAIL");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "FullyPeriodicSquare.vtu");
    Export::TsvExporter::writeMesh(context.getMeshData(), "FullyPeriodicSquare");
    spdlog::info("Mesh exported to FullyPeriodicSquare.vtu");

    return 0;
}
