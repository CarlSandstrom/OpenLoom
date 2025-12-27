#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"

#include <Geom2d_Circle.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Dir2d.hxx>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include <cmath>

using namespace Meshing;

int main()
{
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] %^%v%$");

    spdlog::info("Creating 2D square with circular hole using OpenCASCADE geometry");

    // Create geometry collection
    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();

    // Define square dimensions (10x10 units)
    const double squareSize = 10.0;

    // Define circular hole (center at 5,5, radius 2)
    const double holeRadius = 2.0;
    const double holeCenterX = 5.0;
    const double holeCenterY = 5.0;

    // Create outer square corners using simple Corner2D
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c0", Point2D(0.0, 0.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c1", Point2D(squareSize, 0.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c2", Point2D(squareSize, squareSize)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c3", Point2D(0.0, squareSize)));

    // Create outer square edges using LinearEdge2D
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
        "outer_e0", Point2D(0.0, 0.0), Point2D(squareSize, 0.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
        "outer_e1", Point2D(squareSize, 0.0), Point2D(squareSize, squareSize)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
        "outer_e2", Point2D(squareSize, squareSize), Point2D(0.0, squareSize)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
        "outer_e3", Point2D(0.0, squareSize), Point2D(0.0, 0.0)));

    // Create circular hole using OpenCASCADE Geom2d_Circle
    // We'll split the circle into 4 quadrants to create corners for topology

    gp_Pnt2d center(holeCenterX, holeCenterY);
    gp_Dir2d xDir(1.0, 0.0);
    gp_Ax2d axis(center, xDir);
    gp_Circ2d circle(axis, holeRadius);

    // Create 4 corners on the circle (at 0°, 90°, 180°, 270°)
    const double pi = M_PI;
    std::vector<double> angles = {0.0, pi/2.0, pi, 3.0*pi/2.0};
    std::vector<std::string> cornerIds = {"hole_c0", "hole_c1", "hole_c2", "hole_c3"};

    for (size_t i = 0; i < 4; ++i)
    {
        double x = holeCenterX + holeRadius * std::cos(angles[i]);
        double y = holeCenterY + holeRadius * std::sin(angles[i]);
        gp_Pnt2d point(x, y);
        geometry->addCorner(std::make_unique<Geometry2D::OpenCascade2DCorner>(point, cornerIds[i]));
    }

    // Create 4 circular arc edges using OpenCASCADE trimmed curves
    std::vector<std::string> edgeIds = {"hole_e0", "hole_e1", "hole_e2", "hole_e3"};

    for (size_t i = 0; i < 4; ++i)
    {
        size_t nextIdx = (i + 1) % 4;

        // Create circle curve
        Handle(Geom2d_Circle) circleGeom = new Geom2d_Circle(circle);

        // Trim the circle to create an arc from angle[i] to angle[nextIdx]
        double startParam = angles[i];
        double endParam = angles[nextIdx];

        // Handle wrap-around for last edge
        if (i == 3)
        {
            endParam = 2.0 * pi;
        }

        Handle(Geom2d_TrimmedCurve) arc = new Geom2d_TrimmedCurve(circleGeom, startParam, endParam);

        geometry->addEdge(std::make_unique<Geometry2D::OpenCascade2DEdge>(arc, edgeIds[i]));
    }

    // Create topology
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;

    // Outer square corners
    topoCorners.emplace("outer_c0", Topology2D::Corner2D("outer_c0", {"outer_e0", "outer_e3"}));
    topoCorners.emplace("outer_c1", Topology2D::Corner2D("outer_c1", {"outer_e0", "outer_e1"}));
    topoCorners.emplace("outer_c2", Topology2D::Corner2D("outer_c2", {"outer_e1", "outer_e2"}));
    topoCorners.emplace("outer_c3", Topology2D::Corner2D("outer_c3", {"outer_e2", "outer_e3"}));

    // Circular hole corners
    topoCorners.emplace("hole_c0", Topology2D::Corner2D("hole_c0", {"hole_e0", "hole_e3"}));
    topoCorners.emplace("hole_c1", Topology2D::Corner2D("hole_c1", {"hole_e0", "hole_e1"}));
    topoCorners.emplace("hole_c2", Topology2D::Corner2D("hole_c2", {"hole_e1", "hole_e2"}));
    topoCorners.emplace("hole_c3", Topology2D::Corner2D("hole_c3", {"hole_e2", "hole_e3"}));

    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;

    // Outer square edges (counter-clockwise)
    topoEdges.emplace("outer_e0", Topology2D::Edge2D("outer_e0", "outer_c0", "outer_c1"));
    topoEdges.emplace("outer_e1", Topology2D::Edge2D("outer_e1", "outer_c1", "outer_c2"));
    topoEdges.emplace("outer_e2", Topology2D::Edge2D("outer_e2", "outer_c2", "outer_c3"));
    topoEdges.emplace("outer_e3", Topology2D::Edge2D("outer_e3", "outer_c3", "outer_c0"));

    // Circular hole edges (clockwise - for hole)
    topoEdges.emplace("hole_e0", Topology2D::Edge2D("hole_e0", "hole_c0", "hole_c1"));
    topoEdges.emplace("hole_e1", Topology2D::Edge2D("hole_e1", "hole_c1", "hole_c2"));
    topoEdges.emplace("hole_e2", Topology2D::Edge2D("hole_e2", "hole_c2", "hole_c3"));
    topoEdges.emplace("hole_e3", Topology2D::Edge2D("hole_e3", "hole_c3", "hole_c0"));

    // Boundary edge loop: outer boundary followed by inner boundary (hole)
    std::vector<std::string> boundaryEdgeLoop = {
        "outer_e0", "outer_e1", "outer_e2", "outer_e3",
        "hole_e0", "hole_e1", "hole_e2", "hole_e3"
    };

    auto topology = std::make_unique<Topology2D::Topology2D>(topoCorners, topoEdges, boundaryEdgeLoop);

    // Create meshing context
    MeshingContext2D context(std::move(geometry), std::move(topology));

    // Create constrained Delaunay triangulator
    ConstrainedDelaunay2D mesher(context);

    // Generate constrained mesh
    spdlog::info("Generating constrained Delaunay triangulation with circular hole...");
    mesher.triangulate();

    spdlog::info("Square with circular hole mesh generation complete!");

    return 0;
}
