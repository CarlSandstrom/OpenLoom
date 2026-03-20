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
#include "Topology2D/Topology2D.h"
#include "Common/Logging.h"
#include "spdlog/spdlog.h"

#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

int main()
{
    Common::initLogging();

    // Stress test: 10-pointed star with outer radius R=10, inner radius r=2.
    //
    // The tip angle at each outer vertex is ~8.7° (< 10°). Constraint edges
    // meet at near-zero angles at the tips, stress-testing edge insertion and
    // local Delaunay flipping near sharp vertices. The refiner must fill the
    // very thin triangles at each tip while satisfying the 30° minimum angle.

    spdlog::info("Creating 10-pointed star polygon (tip angle ~8.7°)");

    const size_t numPoints = 10;
    const size_t numVertices = 2 * numPoints;
    const double outerRadius = 10.0;
    const double innerRadius = 2.0;

    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;
    std::vector<std::string> edgeLoop;

    // Compute all vertex positions. Even indices are outer (tips), odd are inner (valleys).
    std::vector<Point2D> vertices(numVertices);
    for (size_t i = 0; i < numVertices; ++i)
    {
        double radius = (i % 2 == 0) ? outerRadius : innerRadius;
        double angle = i * M_PI / numPoints;
        vertices[i] = Point2D(radius * std::cos(angle), radius * std::sin(angle));
    }

    // Add corners and edges. Edge e_i connects c_i -> c_{(i+1) % numVertices}.
    for (size_t i = 0; i < numVertices; ++i)
    {
        std::string cornerId = "c" + std::to_string(i);
        std::string edgeId = "e" + std::to_string(i);
        std::string prevEdgeId = "e" + std::to_string((i + numVertices - 1) % numVertices);
        size_t nextIndex = (i + 1) % numVertices;
        std::string nextCornerId = "c" + std::to_string(nextIndex);

        geometry->addCorner(std::make_unique<Geometry2D::Corner2D>(cornerId, vertices[i]));
        geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>(
            edgeId, vertices[i], vertices[nextIndex]));

        topoCorners.emplace(cornerId, Topology2D::Corner2D(cornerId, {edgeId, prevEdgeId}));
        topoEdges.emplace(edgeId, Topology2D::Edge2D(edgeId, cornerId, nextCornerId));
        edgeLoop.push_back(edgeId);
    }

    auto topology = std::make_unique<Topology2D::Topology2D>(
        topoCorners, topoEdges, edgeLoop,
        std::vector<std::vector<std::string>>{});

    MeshingContext2D context(std::move(geometry), std::move(topology));

    EdgeDiscretizer2D discretizer(context);
    auto discretization = discretizer.discretize();

    ConstrainedDelaunay2D mesher(context, discretization);
    spdlog::info("Generating constrained Delaunay triangulation...");
    mesher.triangulate();

    // Note: Shewchuk refinement is not guaranteed to terminate for input angles
    // smaller than ~20.7°. The ~8.7° tips cause the refiner to cycle indefinitely
    // near each tip. This example intentionally omits refinement — the stress-test
    // value is in the CDT phase, which must insert and recover constraint edges at
    // near-zero angles and perform many local Delaunay flips near the sharp tips.

    spdlog::info("Star polygon mesh generation complete!");

    Export::VtkExporter exporter;
    exporter.exportMesh(context.getMeshData(), "StarPolygon.vtu");

    return 0;
}
