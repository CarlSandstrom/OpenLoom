#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/EdgeDiscretizer2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Topology2D/Topology2D.h"
#include "Common/Logging.h"
#include "spdlog/spdlog.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

int main()
{
    Common::initLogging();

    spdlog::info("Creating 2D rectangle with hole example");

    // Create geometry collection
    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();

    Point2D holeBottomLeft(3.0, 2.5);
    Point2D holeTopRight(7.0, 5.5);
    Point2D holeBottomRight(3.5, 5.0);
    Point2D holeTopLeft(3.0, 5.5);

    // Define outer rectangle corners (10x8 units)
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c0", Point2D(0.0, 0.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c1", Point2D(10.0, 0.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c2", Point2D(10.0, 8.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c3", Point2D(0.0, 8.0)));

    // Define inner rectangle corners (hole: 4x3 units, centered)
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("inner_c0", holeBottomLeft));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("inner_c1", holeBottomRight));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("inner_c2", holeTopRight));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("inner_c3", holeTopLeft));

    // Create outer rectangle edges
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("outer_e0", Point2D(0.0, 0.0), Point2D(10.0, 0.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("outer_e1", Point2D(10.0, 0.0), Point2D(10.0, 8.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("outer_e2", Point2D(10.0, 8.0), Point2D(0.0, 8.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("outer_e3", Point2D(0.0, 8.0), Point2D(0.0, 0.0)));

    // Create inner rectangle edges (hole)
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("inner_e0", holeBottomLeft, holeBottomRight));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("inner_e1", holeBottomRight, holeTopRight));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("inner_e2", holeTopRight, holeTopLeft));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("inner_e3", holeTopLeft, holeBottomLeft));

    // Create topology
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    topoCorners.emplace("outer_c0", Topology2D::Corner2D("outer_c0", {"outer_e0", "outer_e3"}));
    topoCorners.emplace("outer_c1", Topology2D::Corner2D("outer_c1", {"outer_e0", "outer_e1"}));
    topoCorners.emplace("outer_c2", Topology2D::Corner2D("outer_c2", {"outer_e1", "outer_e2"}));
    topoCorners.emplace("outer_c3", Topology2D::Corner2D("outer_c3", {"outer_e2", "outer_e3"}));
    topoCorners.emplace("inner_c0", Topology2D::Corner2D("inner_c0", {"inner_e0", "inner_e3"}));
    topoCorners.emplace("inner_c1", Topology2D::Corner2D("inner_c1", {"inner_e0", "inner_e1"}));
    topoCorners.emplace("inner_c2", Topology2D::Corner2D("inner_c2", {"inner_e1", "inner_e2"}));
    topoCorners.emplace("inner_c3", Topology2D::Corner2D("inner_c3", {"inner_e2", "inner_e3"}));

    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;
    // Outer edges (counter-clockwise)
    topoEdges.emplace("outer_e0", Topology2D::Edge2D("outer_e0", "outer_c0", "outer_c1"));
    topoEdges.emplace("outer_e1", Topology2D::Edge2D("outer_e1", "outer_c1", "outer_c2"));
    topoEdges.emplace("outer_e2", Topology2D::Edge2D("outer_e2", "outer_c2", "outer_c3"));
    topoEdges.emplace("outer_e3", Topology2D::Edge2D("outer_e3", "outer_c3", "outer_c0"));
    // Inner edges (clockwise - for hole)
    topoEdges.emplace("inner_e0", Topology2D::Edge2D("inner_e0", "inner_c0", "inner_c1"));
    topoEdges.emplace("inner_e1", Topology2D::Edge2D("inner_e1", "inner_c1", "inner_c2"));
    topoEdges.emplace("inner_e2", Topology2D::Edge2D("inner_e2", "inner_c2", "inner_c3"));
    topoEdges.emplace("inner_e3", Topology2D::Edge2D("inner_e3", "inner_c3", "inner_c0"));

    // Boundary edge loop: outer boundary followed by inner boundary (hole)
    std::vector<std::string> boundaryEdgeLoop = {
        "outer_e0", "outer_e1", "outer_e2", "outer_e3",
        "inner_e0", "inner_e1", "inner_e2", "inner_e3"};

    auto topology = std::make_unique<Topology2D::Topology2D>(topoCorners, topoEdges, boundaryEdgeLoop);

    // Create meshing context
    MeshingContext2D context(std::move(geometry), std::move(topology));

    // Create constrained Delaunay triangulator
    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();
    ConstrainedDelaunay2D mesher(context, discretization);

    // Generate constrained mesh with 15 samples per edge
    spdlog::info("Generating constrained Delaunay triangulation...");
    mesher.triangulate();

    return 0;
}
