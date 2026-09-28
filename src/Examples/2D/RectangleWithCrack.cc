#include "Common/Logging.h"
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Meshing/Core/2D/BoundaryDiscretizer2D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

int main()
{
    Common::initializeLogging();

    spdlog::info("Creating 2D rectangle with crack example");

    // Create geometry collection
    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();

    Point2D bottomLeft(0, 0);
    Point2D topRight(10, 10);
    Point2D bottomRight(10, 0);
    Point2D topLeft(0, 10);
    Point2D crackStart(0, 5.1);
    Point2D crackTip(9.9, 8.0);
    Point2D crackEnd(0, 4.9);

    // Declare corners
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c0", bottomLeft));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c1", bottomRight));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c2", topRight));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c3", topLeft));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("crack_start", crackStart));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("crack_tip", crackTip));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("crack_end", crackEnd));

    // Create edges
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e0", bottomLeft, bottomRight));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e1", bottomRight, topRight));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e2", topRight, topLeft));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e3", topLeft, crackStart));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("crack_edge1", crackStart, crackTip));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("crack_edge2", crackTip, crackEnd));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e4", crackEnd, bottomLeft));

    // Create topology
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    topologyCorners.emplace("c0", Topology2D::Corner2D("c0", {"e0", "e4"}));
    topologyCorners.emplace("c1", Topology2D::Corner2D("c1", {"e0", "e1"}));
    topologyCorners.emplace("c2", Topology2D::Corner2D("c2", {"e1", "e2"}));
    topologyCorners.emplace("c3", Topology2D::Corner2D("c3", {"e2", "e3"}));
    topologyCorners.emplace("crack_start", Topology2D::Corner2D("crack_start", {"e3", "crack_edge1"}));
    topologyCorners.emplace("crack_tip", Topology2D::Corner2D("crack_tip", {"crack_edge1", "crack_edge2"}));
    topologyCorners.emplace("crack_end", Topology2D::Corner2D("crack_end", {"crack_edge2", "e4"}));

    std::unordered_map<std::string, Topology2D::Edge2D> topologyEdges;
    // Outer edges (counter-clockwise)
    topologyEdges.emplace("e0", Topology2D::Edge2D("e0", "c0", "c1"));
    topologyEdges.emplace("e1", Topology2D::Edge2D("e1", "c1", "c2"));
    topologyEdges.emplace("e2", Topology2D::Edge2D("e2", "c2", "c3"));
    topologyEdges.emplace("e3", Topology2D::Edge2D("e3", "c3", "crack_start"));
    topologyEdges.emplace("crack_edge1", Topology2D::Edge2D("crack_edge1", "crack_start", "crack_tip"));
    topologyEdges.emplace("crack_edge2", Topology2D::Edge2D("crack_edge2", "crack_tip", "crack_end"));
    topologyEdges.emplace("e4", Topology2D::Edge2D("e4", "crack_end", "c0"));

    // Boundary edge loop: outer boundary followed by inner boundary (hole)
    std::vector<std::string> boundaryEdgeLoop = {
        "e0", "e1", "e2", "e3", "crack_edge1", "crack_edge2", "e4"};

    auto topology = std::make_unique<Topology2D::Topology2D>(topologyCorners, topologyEdges, boundaryEdgeLoop);

    // Create meshing context
    MeshingContext2D context(std::move(geometry), std::move(topology));

    // Create constrained Delaunay triangulator
    BoundaryDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    // Generate constrained mesh with 15 samples per edge
    spdlog::info("Generating constrained Delaunay triangulation...");
    ConstrainedDelaunay2D::triangulate(context, discretization, {Point2D(5.0, 3.9)});

    return 0;
}
