#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/EdgeDiscretizer2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/Mesh2DQualitySettings.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Topology2D/Topology2D.h"
#include "Common/Logging.h"
#include "spdlog/spdlog.h"

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

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
        std::string cornerId = prefix + "_c" + std::to_string(i);
        std::string edgeId = prefix + "_e" + std::to_string(i);
        std::string previousEdgeId = prefix + "_e" + std::to_string((i + 3) % 4);

        geometry.addCorner(std::make_unique<Geometry2D::Corner2D>(cornerId, corners[i]));
        geometry.addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
            edgeId, corners[i], corners[(i + 1) % 4]));

        topoCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, previousEdgeId}));
        topoEdges.emplace(edgeId, Topology2D::Edge2D(
                                      edgeId, cornerId,
                                      prefix + "_c" + std::to_string((i + 1) % 4)));
        edgeLoop.push_back(edgeId);
    }
}

void addSpiralPolyline(Geometry2D::GeometryCollection2D& geometry,
                       std::unordered_map<std::string, Topology2D::Corner2D>& topoCorners,
                       std::unordered_map<std::string, Topology2D::Edge2D>& topoEdges,
                       const std::string& prefix,
                       const std::vector<Point2D>& points)
{
    size_t numPoints = points.size();

    for (size_t i = 0; i < numPoints; ++i)
    {
        std::string cornerId = prefix + "_c" + std::to_string(i);

        std::set<std::string> connectedEdges;
        if (i > 0)
            connectedEdges.insert(prefix + "_e" + std::to_string(i - 1));
        if (i < numPoints - 1)
            connectedEdges.insert(prefix + "_e" + std::to_string(i));

        geometry.addCorner(std::make_unique<Geometry2D::Corner2D>(cornerId, points[i]));
        topoCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, connectedEdges));
    }

    for (size_t i = 0; i < numPoints - 1; ++i)
    {
        std::string edgeId = prefix + "_e" + std::to_string(i);
        std::string startCornerId = prefix + "_c" + std::to_string(i);
        std::string endCornerId = prefix + "_c" + std::to_string(i + 1);

        geometry.addEdge(
            std::make_unique<Geometry2D::LinearEdge2D>(edgeId, points[i], points[i + 1]));
        topoEdges.emplace(edgeId, Topology2D::Edge2D(edgeId, startCornerId, endCornerId));
    }
}

int main()
{
    Common::initLogging();

    spdlog::info("Creating 2D spiral channel stress test");

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;

    std::vector<std::string> outerEdgeLoop;
    addSquare(*geometry, topoCorners, topoEdges, outerEdgeLoop, "outer", 10.0);

    // 3-turn rectangular spiral winding inward, arm spacing = 1.0
    // Corridor width between consecutive arms is 1.0 unit throughout
    const std::vector<Point2D> spiralPoints = {
        Point2D(5.0, 2.0),  // p0  — entry
        Point2D(8.0, 2.0),  // p1  — right
        Point2D(8.0, 8.0),  // p2  — up
        Point2D(2.0, 8.0),  // p3  — left
        Point2D(2.0, 3.0),  // p4  — down
        Point2D(7.0, 3.0),  // p5  — right
        Point2D(7.0, 7.0),  // p6  — up
        Point2D(3.0, 7.0),  // p7  — left
        Point2D(3.0, 4.0),  // p8  — down
        Point2D(6.0, 4.0),  // p9  — right
        Point2D(6.0, 6.0),  // p10 — up
        Point2D(4.0, 6.0),  // p11 — left
        Point2D(4.0, 5.0),  // p12 — down
        Point2D(5.0, 5.0),  // p13 — center (end)
    };

    addSpiralPolyline(*geometry, topoCorners, topoEdges, "spiral", spiralPoints);

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges, outerEdgeLoop, std::vector<std::vector<std::string>>{});

    MeshingContext2D context(std::move(geometry), std::move(topology));

    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D mesher(context, discretization);
    spdlog::info("Generating constrained Delaunay triangulation...");
    mesher.triangulate();

    spdlog::info("Refining mesh with ShewchukRefiner2D...");
    ShewchukRefiner2D refiner(context, Mesh2DQualitySettings{});
    refiner.refine();

    spdlog::info("Spiral channel mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "SpiralChannel.vtu");

    return 0;
}
