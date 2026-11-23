#include "ExampleShapes2D.h"
#include "Common/Logging.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Meshing/Core/2D/General/BoundaryDiscretizer2D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/General/MeshingContext2D.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/MeshQualitySettings2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/Base/TriangleElement.h"
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

    spdlog::info("Creating 2D square with circular hole using OpenCASCADE geometry");

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topologyEdges;

    // Add outer square (10x10 units)
    std::vector<std::string> outerEdgeLoop;
    outerEdgeLoop = ExampleShapes2D::addSquare(*geometry, topologyCorners, topologyEdges, "outer", 10.0);

    // Add circular holes - each hole needs its own edge loop
    std::vector<std::string> hole1EdgeLoop;
    hole1EdgeLoop = ExampleShapes2D::addCircle(*geometry, topologyCorners, topologyEdges, "hole1", Point2D(4.0, 5.0), 0.95, 8);

    std::vector<std::string> hole2EdgeLoop;
    hole2EdgeLoop = ExampleShapes2D::addCircle(*geometry, topologyCorners, topologyEdges, "hole2", Point2D(6.0, 5.0), 1.0, 8);

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
    ShewchukRefiner2D refiner(context, Meshing::MeshQualitySettings2D{});
    refiner.refine();

    spdlog::info("Square with circular hole mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "SquareWithCircularHoles.vtu");
    Export::TsvExporter::writeMesh(context.getMeshData(), "SquareWithCircularHoles");

    return 0;
}
