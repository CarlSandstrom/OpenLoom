#include "ExampleShapes2D.h"
#include "Common/Logging.h"
#include "Common/Types.h"
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Meshing/Core/2D/General/BoundaryDiscretizer2D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/General/MeshingContext2D.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/MeshQualitySettings2D.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"


#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace Meshing;

// Stress test: 100x100 square with 25 (5x5 grid) small circular holes.
//
// Circles have radius 5, centered on a 5x5 grid with spacing 16 (centers at 18, 34, 50, 66, 82).
// Gap between adjacent circle edges: 6 units. Gap from the outer boundary: 13 units.
// The CDT must insert and classify 25 independent constraint loops simultaneously,
// stressing constraint edge insertion, boundary proximity handling, and the flood-fill
// hole classifier with many separate excluded regions. The Shewchuk refiner then fills
// the domain (the channels between circles and the outer boundary) with quality triangles.

namespace
{
constexpr double SQUARE_SIZE = 100.0;
constexpr double CIRCLE_RADIUS = 5.0;
constexpr std::size_t SEGMENTS_PER_CIRCLE = 16;
constexpr std::size_t GRID_SIZE = 5;
constexpr double GRID_SPACING = 16.0;
constexpr double START_OFFSET = (SQUARE_SIZE - (GRID_SIZE - 1) * GRID_SPACING) / 2.0;

struct DomainDescription
{
    std::unique_ptr<Geometry2D::GeometryCollection2D> geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topologyEdges;
};

std::vector<std::vector<std::string>> addCircleGrid(DomainDescription& domain)
{
    std::vector<std::vector<std::string>> holeEdgeLoops;
    std::size_t circleIndex = 0;
    for (std::size_t row = 0; row < GRID_SIZE; ++row)
    {
        for (std::size_t column = 0; column < GRID_SIZE; ++column)
        {
            auto center = Point2D(START_OFFSET + column * GRID_SPACING, START_OFFSET + row * GRID_SPACING);
            auto prefix = "circle" + std::to_string(circleIndex);

            spdlog::info("Adding circle {}: center ({:.1f}, {:.1f}), radius {:.1f}",
                         circleIndex, center.x(), center.y(), CIRCLE_RADIUS);

            holeEdgeLoops.push_back(ExampleShapes2D::addCircle(*domain.geometry, domain.topologyCorners, domain.topologyEdges,
                                                               prefix, center, CIRCLE_RADIUS, SEGMENTS_PER_CIRCLE));
            ++circleIndex;
        }
    }
    return holeEdgeLoops;
}
} // namespace

int main()
{
    Common::initializeLogging();

    spdlog::info("Creating many circles in square stress test");

    DomainDescription domain;
    auto outerEdgeLoop = ExampleShapes2D::addSquare(*domain.geometry, domain.topologyCorners, domain.topologyEdges, "outer", SQUARE_SIZE);
    auto holeEdgeLoops = addCircleGrid(domain);

    spdlog::info("Added {} circular holes", holeEdgeLoops.size());

    auto topology = std::make_unique<Topology2D::Topology2D>(
        domain.topologyCorners, domain.topologyEdges, outerEdgeLoop, holeEdgeLoops);

    MeshingContext2D context(std::move(domain.geometry), std::move(topology));

    BoundaryDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    spdlog::info("Generating constrained Delaunay triangulation...");
    ConstrainedDelaunay2D::triangulate(context, discretization);

    spdlog::info("Refining mesh with ShewchukRefiner2D...");
    ShewchukRefiner2D refiner(context, MeshQualitySettings2D{});
    refiner.refine();

    spdlog::info("Many circles in square mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "ManyCirclesInSquare.vtu");

    return 0;
}
