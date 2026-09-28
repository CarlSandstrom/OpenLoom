#include "Common/Logging.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"
#include "Meshing/Core/2D/BoundaryDiscretizer2D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/Mesh2DQualitySettings.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"

#include <Geom2d_Circle.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt2d.hxx>

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

// Helper to add a square boundary to geometry and topology
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

// Helper to add a circular hole to geometry and topology
void addCircularHole(Geometry2D::GeometryCollection2D& geometry,
                     std::unordered_map<std::string, Topology2D::Corner2D>& topologyCorners,
                     std::unordered_map<std::string, Topology2D::Edge2D>& topologyEdges,
                     std::vector<std::string>& edgeLoop,
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
        edgeLoop.push_back(edgeId);
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

int main()
{
    Common::initializeLogging();

    spdlog::info("Creating 2D square with circular hole using OpenCASCADE geometry");

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topologyEdges;

    // Add outer square (10x10 units)
    std::vector<std::string> outerEdgeLoop;
    addSquare(*geometry, topologyCorners, topologyEdges, outerEdgeLoop, "outer", 10.0);

    // Add circular holes - each hole needs its own edge loop
    std::vector<std::string> hole1EdgeLoop;
    addCircularHole(*geometry, topologyCorners, topologyEdges, hole1EdgeLoop, "hole1", 4.0, 5.0, 0.95, 8);

    std::vector<std::string> hole2EdgeLoop;
    addCircularHole(*geometry, topologyCorners, topologyEdges, hole2EdgeLoop, "hole2", 6.0, 5.0, 1.0, 8);

    auto topology = std::make_unique<Topology2D::Topology2D>(topologyCorners, topologyEdges, outerEdgeLoop,
                                                             std::vector<std::vector<std::string>>{hole1EdgeLoop, hole2EdgeLoop});

    // Create meshing context and triangulate
    MeshingContext2D context(std::move(geometry), std::move(topology));

    // Discretize edges
    BoundaryDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    // Create constrained Delaunay triangulator
    spdlog::info("Generating constrained Delaunay triangulation with circular hole...");
    ConstrainedDelaunay2D::triangulate(context, discretization);

    // Refine mesh with ShewchukRefiner2D
    spdlog::info("Refining mesh with ShewchukRefiner2D...");
    ShewchukRefiner2D refiner(context, Meshing::Mesh2DQualitySettings{});
    refiner.refine();

    spdlog::info("Square with circular hole mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "SquareWithCircularHoles.vtu");
    Export::TsvExporter::writeMesh(context.getMeshData(), "SquareWithCircularHoles");

    return 0;
}
