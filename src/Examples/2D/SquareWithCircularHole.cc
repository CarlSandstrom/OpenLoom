#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/EdgeDiscretizer2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/Shewchuk2DQualityController.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
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

// Helper to add a circular hole to geometry and topology
void addCircularHole(Geometry2D::GeometryCollection2D& geometry,
                     std::unordered_map<std::string, Topology2D::Corner2D>& topoCorners,
                     std::unordered_map<std::string, Topology2D::Edge2D>& topoEdges,
                     std::vector<std::string>& edgeLoop,
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
        edgeLoop.push_back(eId);
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

int main()
{
    spdlog::set_level(spdlog::level::debug);
    spdlog::set_pattern("[%H:%M:%S] %^%v%$");

    spdlog::info("Creating 2D square with circular hole using OpenCASCADE geometry");

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;

    // Add outer square (10x10 units)
    std::vector<std::string> outerEdgeLoop;
    addSquare(*geometry, topoCorners, topoEdges, outerEdgeLoop, "outer", 10.0);

    // Add circular holes - each hole needs its own edge loop
    std::vector<std::string> hole1EdgeLoop;
    addCircularHole(*geometry, topoCorners, topoEdges, hole1EdgeLoop, "hole1", 4.0, 5.0, 0.95, 8);

    std::vector<std::string> hole2EdgeLoop;
    addCircularHole(*geometry, topoCorners, topoEdges, hole2EdgeLoop, "hole2", 6.0, 5.0, 1.0, 8);

    auto topology = std::make_unique<Topology2D::Topology2D>(topoCorners, topoEdges, outerEdgeLoop,
                                                             std::vector<std::vector<std::string>>{hole1EdgeLoop, hole2EdgeLoop});

    // Create meshing context and triangulate
    MeshingContext2D context(std::move(geometry), std::move(topology));

    // Discretize edges
    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    // Create constrained Delaunay triangulator
    ConstrainedDelaunay2D mesher(context, discretization);
    spdlog::info("Generating constrained Delaunay triangulation with circular hole...");
    mesher.triangulate();

    // Refine mesh with ShewchukRefiner2D
    spdlog::info("Refining mesh with ShewchukRefiner2D...");
    Shewchuk2DQualityController qualityController(context.getMeshData(),
                                                  2.0,        // Max circumradius to shortest edge ratio
                                                  M_PI / 6.0, // Min angle 30 degrees
                                                  10000);     // Max elements
    ShewchukRefiner2D refiner(context, qualityController);
    refiner.refine();

    spdlog::info("Square with circular hole mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "SquareWithCircularHoles.vtu");

    return 0;
}
