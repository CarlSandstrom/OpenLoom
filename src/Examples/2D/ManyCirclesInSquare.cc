#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"
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

// Stress test: 100x100 square with 25 (5x5 grid) small circular holes.
//
// Circles have radius 5, centered on a 5x5 grid with spacing 16 (centers at 18, 34, 50, 66, 82).
// Gap between adjacent circle edges: 6 units. Gap from the outer boundary: 13 units.
// The CDT must insert and classify 25 independent constraint loops simultaneously,
// stressing constraint edge insertion, boundary proximity handling, and the flood-fill
// hole classifier with many separate excluded regions. The Shewchuk refiner then fills
// the domain (the channels between circles and the outer boundary) with quality triangles.

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
        std::string cornerId = prefix + "_c" + std::to_string(i);
        std::string edgeId = prefix + "_e" + std::to_string(i);
        std::string prevEdgeId = prefix + "_e" + std::to_string((i + 3) % 4);

        geometry.addCorner(std::make_unique<Geometry2D::Corner2D>(cornerId, corners[i]));
        geometry.addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
            edgeId, corners[i], corners[(i + 1) % 4]));

        topoCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, prevEdgeId}));
        topoEdges.emplace(edgeId, Topology2D::Edge2D(
                                      edgeId, cornerId, prefix + "_c" + std::to_string((i + 1) % 4)));
        edgeLoop.push_back(edgeId);
    }
}

void addCircularConstraint(Geometry2D::GeometryCollection2D& geometry,
                           std::unordered_map<std::string, Topology2D::Corner2D>& topoCorners,
                           std::unordered_map<std::string, Topology2D::Edge2D>& topoEdges,
                           const std::string& prefix,
                           double centerX,
                           double centerY,
                           double radius,
                           size_t numSegments)
{
    gp_Pnt2d center(centerX, centerY);
    gp_Ax2d axis(center, gp_Dir2d(1.0, 0.0));
    gp_Circ2d circle(axis, radius);

    for (size_t i = 0; i < numSegments; ++i)
    {
        double angle = 2.0 * M_PI * i / numSegments;
        double x = centerX + radius * std::cos(angle);
        double y = centerY + radius * std::sin(angle);

        std::string cornerId = prefix + "_c" + std::to_string(i);
        std::string edgeId = prefix + "_e" + std::to_string(i);
        std::string prevEdgeId = prefix + "_e" + std::to_string((i + numSegments - 1) % numSegments);

        geometry.addCorner(
            std::make_unique<Geometry2D::OpenCascade2DCorner>(gp_Pnt2d(x, y), cornerId));
        topoCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, prevEdgeId}));
    }

    for (size_t i = 0; i < numSegments; ++i)
    {
        double startAngle = 2.0 * M_PI * i / numSegments;
        double endAngle = 2.0 * M_PI * (i + 1) / numSegments;

        Handle(Geom2d_Circle) circleGeom = new Geom2d_Circle(circle);
        Handle(Geom2d_TrimmedCurve) arc =
            new Geom2d_TrimmedCurve(circleGeom, startAngle, endAngle);

        std::string edgeId = prefix + "_e" + std::to_string(i);
        std::string startCornerId = prefix + "_c" + std::to_string(i);
        std::string endCornerId = prefix + "_c" + std::to_string((i + 1) % numSegments);

        geometry.addEdge(std::make_unique<Geometry2D::OpenCascade2DEdge>(arc, edgeId));
        topoEdges.emplace(edgeId, Topology2D::Edge2D(edgeId, startCornerId, endCornerId));
    }
}

int main()
{
    Common::initLogging();

    spdlog::info("Creating many circles in square stress test");

    const double squareSize = 100.0;
    const double circleRadius = 5.0;
    const size_t numSegments = 16;
    const size_t gridN = 5;
    const double spacing = 16.0;
    // Center the 5x5 grid: total span = 4 * 16 = 64, start = (100 - 64) / 2 = 18
    const double startOffset = (squareSize - (gridN - 1) * spacing) / 2.0;

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;

    std::vector<std::string> outerEdgeLoop;
    addSquare(*geometry, topoCorners, topoEdges, outerEdgeLoop, "outer", squareSize);

    std::vector<std::vector<std::string>> holeEdgeLoops;
    size_t circleIndex = 0;
    for (size_t row = 0; row < gridN; ++row)
    {
        for (size_t col = 0; col < gridN; ++col)
        {
            double centerX = startOffset + col * spacing;
            double centerY = startOffset + row * spacing;
            std::string prefix = "circle" + std::to_string(circleIndex);

            spdlog::info("Adding circle {}: center ({:.1f}, {:.1f}), radius {:.1f}",
                         circleIndex, centerX, centerY, circleRadius);

            addCircularConstraint(
                *geometry, topoCorners, topoEdges, prefix, centerX, centerY, circleRadius, numSegments);

            std::vector<std::string> holeLoop;
            for (size_t i = 0; i < numSegments; ++i)
                holeLoop.push_back(prefix + "_e" + std::to_string(i));
            holeEdgeLoops.push_back(std::move(holeLoop));

            ++circleIndex;
        }
    }

    spdlog::info("Added {} circular holes", holeEdgeLoops.size());

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges, outerEdgeLoop, holeEdgeLoops);

    MeshingContext2D context(std::move(geometry), std::move(topology));

    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D mesher(context, discretization);
    spdlog::info("Generating constrained Delaunay triangulation...");
    mesher.triangulate();

    spdlog::info("Refining mesh with ShewchukRefiner2D...");
    ShewchukRefiner2D refiner(context, Meshing::Mesh2DQualitySettings{});
    refiner.refine();

    spdlog::info("Many circles in square mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "ManyCirclesInSquare.vtu");

    return 0;
}
