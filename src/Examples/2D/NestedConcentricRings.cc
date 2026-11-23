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

// Stress test: 20x20 square with 4 concentric circles as internal constraints.
//
// Radii decrease by factor 0.9 per ring: 8.0, 7.2, 6.48, 5.83 (all centered at (10,10)).
// This produces 4 rings of 32 arc segments each packed into a ~2.2-unit wide band,
// with minimum channel width ~0.65 units between the two innermost rings.
// The CDT must insert and recover 128 nearly-touching arc segments, stressing constraint
// edge insertion and local Delaunay flipping near tightly packed boundaries.
// The Shewchuk refiner must then fill narrow annular channels satisfying the 30° criterion.

int main()
{
    Common::initializeLogging();

    spdlog::info("Creating nested concentric rings stress test");

    const double squareSize = 20.0;
    const double centerX = 10.0;
    const double centerY = 10.0;
    const size_t numberOfSegments = 32;

    // Four concentric rings with radii decreasing by factor 0.9.
    // Channel widths: 0.80, 0.72, 0.65 units (innermost is tightest).
    const std::vector<double> radii = {8.00, 7.20, 6.48, 5.83};

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topologyEdges;

    std::vector<std::string> outerEdgeLoop;
    outerEdgeLoop = ExampleShapes2D::addSquare(*geometry, topologyCorners, topologyEdges, "outer", squareSize);

    for (size_t index = 0; index < radii.size(); ++index)
    {
        std::string prefix = "ring" + std::to_string(index + 1);
        spdlog::info("Adding circle constraint {}: radius {:.2f}", index + 1, radii[index]);
        ExampleShapes2D::addCircle(
            *geometry, topologyCorners, topologyEdges, prefix, Point2D(centerX, centerY), radii[index], numberOfSegments);
    }

    // No hole loops: circles are internal constraints, not excluded regions.
    // The entire 20x20 square is the domain; the circles partition it into
    // nested annular channels that the mesher must fill.
    auto topology = std::make_unique<Topology2D::Topology2D>(
        topologyCorners, topologyEdges, outerEdgeLoop,
        std::vector<std::vector<std::string>>{});

    MeshingContext2D context(std::move(geometry), std::move(topology));

    BoundaryDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    spdlog::info("Generating constrained Delaunay triangulation...");
    ConstrainedDelaunay2D::triangulate(context, discretization);

    spdlog::info("Refining mesh with ShewchukRefiner2D...");
    ShewchukRefiner2D refiner(context, Meshing::MeshQualitySettings2D{});
    refiner.refine();

    spdlog::info("Nested concentric rings mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "NestedConcentricRings.vtu");
    Export::TsvExporter::writeMesh(context.getMeshData(), "NestedConcentricRings");

    return 0;
}
