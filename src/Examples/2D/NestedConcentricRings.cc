#include "Export/VtkExporter.h"
#include "Geometry/2D/Base/Corner2D.h"
#include "Geometry/2D/Base/GeometryCollection2D.h"
#include "Geometry/2D/Base/LinearEdge2D.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DCorner.h"
#include "Geometry/2D/OpenCascade/OpenCascade2DEdge.h"
#include "Meshing/Core/2D/ConstrainedDelaunay2D.h"
#include "Meshing/Core/2D/EdgeDiscretizer2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/2D/Mesh2DQualitySettings.h"
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

// Stress test: 20x20 square with 4 concentric circles as internal constraints.
//
// Radii decrease by factor 0.9 per ring: 8.0, 7.2, 6.48, 5.83 (all centered at (10,10)).
// This produces 4 rings of 32 arc segments each packed into a ~2.2-unit wide band,
// with minimum channel width ~0.65 units between the two innermost rings.
// The CDT must insert and recover 128 nearly-touching arc segments, stressing constraint
// edge insertion and local Delaunay flipping near tightly packed boundaries.
// The Shewchuk refiner must then fill narrow annular channels satisfying the 30° criterion.

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

// Add a circle as an internal constraint (not a hole boundary).
// The flood-fill classifier does not stop at these edges, so the full square
// domain is meshed and the circles create constrained internal ring channels.
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

    spdlog::info("Creating nested concentric rings stress test");

    const double squareSize = 20.0;
    const double centerX = 10.0;
    const double centerY = 10.0;
    const size_t numSegments = 32;

    // Four concentric rings with radii decreasing by factor 0.9.
    // Channel widths: 0.80, 0.72, 0.65 units (innermost is tightest).
    const std::vector<double> radii = {8.00, 7.20, 6.48, 5.83};

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;

    std::vector<std::string> outerEdgeLoop;
    addSquare(*geometry, topoCorners, topoEdges, outerEdgeLoop, "outer", squareSize);

    for (size_t index = 0; index < radii.size(); ++index)
    {
        std::string prefix = "ring" + std::to_string(index + 1);
        spdlog::info("Adding circle constraint {}: radius {:.2f}", index + 1, radii[index]);
        addCircularConstraint(
            *geometry, topoCorners, topoEdges, prefix, centerX, centerY, radii[index], numSegments);
    }

    // No hole loops: circles are internal constraints, not excluded regions.
    // The entire 20x20 square is the domain; the circles partition it into
    // nested annular channels that the mesher must fill.
    auto topology = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges, outerEdgeLoop,
        std::vector<std::vector<std::string>>{});

    MeshingContext2D context(std::move(geometry), std::move(topology));

    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D mesher(context, discretization);
    spdlog::info("Generating constrained Delaunay triangulation...");
    mesher.triangulate();

    spdlog::info("Refining mesh with ShewchukRefiner2D...");
    ShewchukRefiner2D refiner(context, Meshing::Mesh2DQualitySettings{});
    refiner.refine();

    spdlog::info("Nested concentric rings mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "NestedConcentricRings.vtu");

    return 0;
}
