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

    // Stress test: 200x2 rectangle (aspect ratio 100:1) with a 20x0.8 interior hole.
    //
    // The long edges produce nearly-collinear constraint point sequences, stressing
    // CDT edge insertion. The initial triangulation generates near-degenerate triangles
    // (angles approaching 0° and 180°). The Shewchuk refiner must produce many Steiner
    // points to achieve the 30° minimum angle criterion across the full 200-unit span.
    // The hole adds constraint edges close to the outer long walls (0.6 unit gap),
    // further stressing insertion near tightly packed boundaries.

    spdlog::info("Creating thin sliver domain (200x2, aspect ratio 100:1) with interior hole");

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();

    // Outer boundary: 200x2 rectangle
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c0", Point2D(0.0, 0.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c1", Point2D(200.0, 0.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c2", Point2D(200.0, 2.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c3", Point2D(0.0, 2.0)));

    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("outer_e0", Point2D(0.0, 0.0), Point2D(200.0, 0.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("outer_e1", Point2D(200.0, 0.0), Point2D(200.0, 2.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("outer_e2", Point2D(200.0, 2.0), Point2D(0.0, 2.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("outer_e3", Point2D(0.0, 2.0), Point2D(0.0, 0.0)));

    // Interior hole: 20x0.8 rectangle centered at (70, 1.0), 0.6 units from each long wall
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("hole_c0", Point2D(60.0, 0.6)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("hole_c1", Point2D(80.0, 0.6)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("hole_c2", Point2D(80.0, 1.4)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("hole_c3", Point2D(60.0, 1.4)));

    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("hole_e0", Point2D(60.0, 0.6), Point2D(80.0, 0.6)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("hole_e1", Point2D(80.0, 0.6), Point2D(80.0, 1.4)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("hole_e2", Point2D(80.0, 1.4), Point2D(60.0, 1.4)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("hole_e3", Point2D(60.0, 1.4), Point2D(60.0, 0.6)));

    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    topoCorners.emplace("outer_c0", Topology2D::Corner2D("outer_c0", {"outer_e0", "outer_e3"}));
    topoCorners.emplace("outer_c1", Topology2D::Corner2D("outer_c1", {"outer_e0", "outer_e1"}));
    topoCorners.emplace("outer_c2", Topology2D::Corner2D("outer_c2", {"outer_e1", "outer_e2"}));
    topoCorners.emplace("outer_c3", Topology2D::Corner2D("outer_c3", {"outer_e2", "outer_e3"}));
    topoCorners.emplace("hole_c0", Topology2D::Corner2D("hole_c0", {"hole_e0", "hole_e3"}));
    topoCorners.emplace("hole_c1", Topology2D::Corner2D("hole_c1", {"hole_e0", "hole_e1"}));
    topoCorners.emplace("hole_c2", Topology2D::Corner2D("hole_c2", {"hole_e1", "hole_e2"}));
    topoCorners.emplace("hole_c3", Topology2D::Corner2D("hole_c3", {"hole_e2", "hole_e3"}));

    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;
    topoEdges.emplace("outer_e0", Topology2D::Edge2D("outer_e0", "outer_c0", "outer_c1"));
    topoEdges.emplace("outer_e1", Topology2D::Edge2D("outer_e1", "outer_c1", "outer_c2"));
    topoEdges.emplace("outer_e2", Topology2D::Edge2D("outer_e2", "outer_c2", "outer_c3"));
    topoEdges.emplace("outer_e3", Topology2D::Edge2D("outer_e3", "outer_c3", "outer_c0"));
    topoEdges.emplace("hole_e0", Topology2D::Edge2D("hole_e0", "hole_c0", "hole_c1"));
    topoEdges.emplace("hole_e1", Topology2D::Edge2D("hole_e1", "hole_c1", "hole_c2"));
    topoEdges.emplace("hole_e2", Topology2D::Edge2D("hole_e2", "hole_c2", "hole_c3"));
    topoEdges.emplace("hole_e3", Topology2D::Edge2D("hole_e3", "hole_c3", "hole_c0"));

    std::vector<std::string> outerEdgeLoop = {"outer_e0", "outer_e1", "outer_e2", "outer_e3"};
    std::vector<std::string> holeEdgeLoop = {"hole_e0", "hole_e1", "hole_e2", "hole_e3"};

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges, outerEdgeLoop,
        std::vector<std::vector<std::string>>{holeEdgeLoop});

    MeshingContext2D context(std::move(geometry), std::move(topology));

    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D mesher(context, discretization);
    spdlog::info("Generating constrained Delaunay triangulation...");
    mesher.triangulate();

    // Raise the element limit: a 200x2 domain with 30° quality requires many Steiner points.
    Meshing::Mesh2DQualitySettings qualitySettings;
    qualitySettings.elementLimit = 50000;

    spdlog::info("Refining mesh with ShewchukRefiner2D...");
    ShewchukRefiner2D refiner(context, qualitySettings);
    refiner.refine();

    spdlog::info("Thin sliver domain mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "ThinSliverDomain.vtu");

    return 0;
}
