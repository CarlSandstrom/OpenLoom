#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/EdgeDiscretizer2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/Mesh2DQualitySettings.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Topology2D/Topology2D.h"
#include "Common/Logging.h"
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

// Helper to add a circular internal edge to geometry and topology
// Unlike addCircularHole, these edges are constrained but do not define a hole
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

int main()
{
    Common::initLogging();

    spdlog::info("Creating 2D square with internal circular edges (not holes)");

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;

    // Add outer square (10x10 units)
    std::vector<std::string> outerEdgeLoop;
    addSquare(*geometry, topoCorners, topoEdges, outerEdgeLoop, "outer", 10.0);

    // Add circular internal edges - these are constrained but NOT holes
    addCircularEdge(*geometry, topoCorners, topoEdges, "circle1", 4.0, 5.0, 0.95, 8);
    addCircularEdge(*geometry, topoCorners, topoEdges, "circle2", 6.0, 5.0, 1.0, 8);

    // No hole edge loops - circles are internal constraints, not holes
    auto topology = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges, outerEdgeLoop,
        std::vector<std::vector<std::string>>{});

    // Create meshing context and triangulate
    MeshingContext2D context(std::move(geometry), std::move(topology));

    // Discretize edges
    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    // Create constrained Delaunay triangulator
    ConstrainedDelaunay2D mesher(context, discretization);
    spdlog::info("Generating constrained Delaunay triangulation with internal circular edges...");
    mesher.triangulate();

    // Refine mesh with ShewchukRefiner2D
    spdlog::info("Refining mesh with ShewchukRefiner2D...");
    ShewchukRefiner2D refiner(context, Meshing::Mesh2DQualitySettings{});
    refiner.refine();

    spdlog::info("Square with internal circular edges mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "SquareWithInternalCircles.vtu");

    return 0;
}
