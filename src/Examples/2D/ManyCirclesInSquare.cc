#include "Common/Logging.h"
#include "Common/Types.h"
#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"
#include "Meshing/Core/2D/BoundaryDiscretizer2D.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/ShewchukRefiner2D.h"
#include "Meshing/Data/2D/Mesh2DQualitySettings.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"

#include <Geom2d_Circle.hxx>
#include <Geom2d_TrimmedCurve.hxx>
#include <gp_Ax2d.hxx>
#include <gp_Circ2d.hxx>
#include <gp_Dir2d.hxx>
#include <gp_Pnt2d.hxx>

#include <cmath>
#include <cstddef>
#include <memory>
#include <numbers>
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
constexpr std::size_t SQUARE_CORNER_COUNT = 4;
constexpr double FULL_TURN = 2.0 * std::numbers::pi;
constexpr double START_OFFSET = (SQUARE_SIZE - (GRID_SIZE - 1) * GRID_SPACING) / 2.0;

struct DomainDescription
{
    std::unique_ptr<Geometry2D::GeometryCollection2D> geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topologyCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topologyEdges;
};

std::string cornerIdentifier(const std::string& prefix, std::size_t index)
{
    return prefix + "_c" + std::to_string(index);
}

std::string edgeIdentifier(const std::string& prefix, std::size_t index)
{
    return prefix + "_e" + std::to_string(index);
}

std::vector<std::string> addSquare(DomainDescription& domain, const std::string& prefix, double size)
{
    auto origin = Point2D(0.0, 0.0);
    std::vector<Point2D> corners = {
        origin,
        Point2D(origin.x() + size, origin.y()),
        Point2D(origin.x() + size, origin.y() + size),
        Point2D(origin.x(), origin.y() + size)};

    std::vector<std::string> edgeLoop;
    for (std::size_t index = 0; index < SQUARE_CORNER_COUNT; ++index)
    {
        auto nextIndex = (index + 1) % SQUARE_CORNER_COUNT;
        auto cornerId = cornerIdentifier(prefix, index);
        auto edgeId = edgeIdentifier(prefix, index);
        auto previousEdgeId = edgeIdentifier(prefix, (index + SQUARE_CORNER_COUNT - 1) % SQUARE_CORNER_COUNT);

        domain.geometry->addCorner(std::make_unique<Geometry2D::Corner2D>(cornerId, corners[index]));
        domain.geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>(edgeId, corners[index], corners[nextIndex]));

        domain.topologyCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, previousEdgeId}));
        domain.topologyEdges.emplace(edgeId, Topology2D::Edge2D(edgeId, cornerId, cornerIdentifier(prefix, nextIndex)));
        edgeLoop.push_back(edgeId);
    }
    return edgeLoop;
}

std::vector<std::string> addCircularConstraint(DomainDescription& domain,
                                               const std::string& prefix,
                                               const Point2D& center,
                                               double radius,
                                               std::size_t numberOfSegments)
{
    gp_Pnt2d circleCenter(center.x(), center.y());
    gp_Ax2d axis(circleCenter, gp_Dir2d(1.0, 0.0));
    gp_Circ2d circle(axis, radius);

    for (std::size_t index = 0; index < numberOfSegments; ++index)
    {
        auto angle = FULL_TURN * index / numberOfSegments;
        auto x = center.x() + radius * std::cos(angle);
        auto y = center.y() + radius * std::sin(angle);

        auto cornerId = cornerIdentifier(prefix, index);
        auto edgeId = edgeIdentifier(prefix, index);
        auto previousEdgeId = edgeIdentifier(prefix, (index + numberOfSegments - 1) % numberOfSegments);

        domain.geometry->addCorner(std::make_unique<Geometry2D::OpenCascade2DCorner>(gp_Pnt2d(x, y), cornerId));
        domain.topologyCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, previousEdgeId}));
    }

    std::vector<std::string> edgeLoop;
    for (std::size_t index = 0; index < numberOfSegments; ++index)
    {
        auto startAngle = FULL_TURN * index / numberOfSegments;
        auto endAngle = FULL_TURN * (index + 1) / numberOfSegments;

        Handle(Geom2d_Circle) circleGeometry = new Geom2d_Circle(circle);
        Handle(Geom2d_TrimmedCurve) arc = new Geom2d_TrimmedCurve(circleGeometry, startAngle, endAngle);

        auto edgeId = edgeIdentifier(prefix, index);
        auto startCornerId = cornerIdentifier(prefix, index);
        auto endCornerId = cornerIdentifier(prefix, (index + 1) % numberOfSegments);

        domain.geometry->addEdge(std::make_unique<Geometry2D::OpenCascade2DEdge>(arc, edgeId));
        domain.topologyEdges.emplace(edgeId, Topology2D::Edge2D(edgeId, startCornerId, endCornerId));
        edgeLoop.push_back(edgeId);
    }
    return edgeLoop;
}

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

            holeEdgeLoops.push_back(addCircularConstraint(domain, prefix, center, CIRCLE_RADIUS, SEGMENTS_PER_CIRCLE));
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
    auto outerEdgeLoop = addSquare(domain, "outer", SQUARE_SIZE);
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
    ShewchukRefiner2D refiner(context, Mesh2DQualitySettings{});
    refiner.refine();

    spdlog::info("Many circles in square mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "ManyCirclesInSquare.vtu");

    return 0;
}
