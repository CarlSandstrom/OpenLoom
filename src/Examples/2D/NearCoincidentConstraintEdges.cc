#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/EdgeDiscretizer2D.h"
#include "Meshing/Core/2D/Mesh2DQualitySettings.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
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

    // Stress test: 100x100 square with a very thin crack entering from the left boundary.
    //
    // The crack runs from (0, 50.5) and (0, 49.5) on the left edge to a shared tip at
    // (90, 50.0). The two crack-wall constraint edges are nearly coincident — they diverge
    // by only 1 unit over 90 units of length (aspect ratio 90:1), meeting at the tip with
    // an interior angle of ~0.64°. At the tip, the two edges are nearly parallel, producing
    // a near-degenerate vertex configuration. The crack interior (the thin triangular region)
    // is included in the mesh. This stresses CDT constraint edge insertion near nearly-
    // coincident segments and the Shewchuk refiner's behaviour adjacent to the acute tip.

    spdlog::info("Creating near-coincident constraint edges stress test");

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();

    // Outer square corners
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c0", Point2D(0.0, 0.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c1", Point2D(100.0, 0.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c2", Point2D(100.0, 100.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c3", Point2D(0.0, 100.0)));

    // Crack corners
    // c_upper: upper entry point of crack on left boundary (0, 50.5)
    // c_tip:   crack tip where the two walls nearly coincide (90, 50.0)
    // c_lower: lower entry point of crack on left boundary (0, 49.5)
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c_upper", Point2D(0.0, 50.5)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c_tip",   Point2D(90.0, 50.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("c_lower", Point2D(0.0, 49.5)));

    // Outer square edges (counter-clockwise)
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e0", Point2D(0.0, 0.0),     Point2D(100.0, 0.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e1", Point2D(100.0, 0.0),   Point2D(100.0, 100.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e2", Point2D(100.0, 100.0), Point2D(0.0, 100.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e3", Point2D(0.0, 100.0),   Point2D(0.0, 50.5)));

    // Crack wall edges — nearly coincident, meeting at ~0.64° at the tip
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e_upper", Point2D(0.0, 50.5), Point2D(90.0, 50.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e_lower", Point2D(90.0, 50.0), Point2D(0.0, 49.5)));

    // Left-lower segment from crack bottom entry to domain origin
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("e4", Point2D(0.0, 49.5), Point2D(0.0, 0.0)));

    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    topoCorners.emplace("c0",      Topology2D::Corner2D("c0",      {"e0",      "e4"}));
    topoCorners.emplace("c1",      Topology2D::Corner2D("c1",      {"e1",      "e0"}));
    topoCorners.emplace("c2",      Topology2D::Corner2D("c2",      {"e2",      "e1"}));
    topoCorners.emplace("c3",      Topology2D::Corner2D("c3",      {"e3",      "e2"}));
    topoCorners.emplace("c_upper", Topology2D::Corner2D("c_upper", {"e_upper", "e3"}));
    topoCorners.emplace("c_tip",   Topology2D::Corner2D("c_tip",   {"e_lower", "e_upper"}));
    topoCorners.emplace("c_lower", Topology2D::Corner2D("c_lower", {"e4",      "e_lower"}));

    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;
    topoEdges.emplace("e0",      Topology2D::Edge2D("e0",      "c0",      "c1"));
    topoEdges.emplace("e1",      Topology2D::Edge2D("e1",      "c1",      "c2"));
    topoEdges.emplace("e2",      Topology2D::Edge2D("e2",      "c2",      "c3"));
    topoEdges.emplace("e3",      Topology2D::Edge2D("e3",      "c3",      "c_upper"));
    topoEdges.emplace("e_upper", Topology2D::Edge2D("e_upper", "c_upper", "c_tip"));
    topoEdges.emplace("e_lower", Topology2D::Edge2D("e_lower", "c_tip",   "c_lower"));
    topoEdges.emplace("e4",      Topology2D::Edge2D("e4",      "c_lower", "c0"));

    std::vector<std::string> boundaryEdgeLoop = {"e0", "e1", "e2", "e3", "e_upper", "e_lower", "e4"};

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges, boundaryEdgeLoop);

    MeshingContext2D context(std::move(geometry), std::move(topology));

    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D mesher(context, discretization);
    spdlog::info("Generating constrained Delaunay triangulation...");
    mesher.triangulate();

    spdlog::info("Refining mesh with ShewchukRefiner2D...");
    ShewchukRefiner2D refiner(context, Meshing::Mesh2DQualitySettings{});
    refiner.refine();

    spdlog::info("Near-coincident constraint edges mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "NearCoincidentConstraintEdges.vtu");

    return 0;
}
