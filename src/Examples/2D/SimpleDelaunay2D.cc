#include "Common/Logging.h"
#include "Common/Types.h"
#include "Export/TsvExporter.h"
#include "Export/VtkExporter.h"
#include "Meshing/Core/2D/Delaunay2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "spdlog/spdlog.h"

#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

using namespace Meshing;

namespace
{
constexpr double ANGLE_STEP = std::numbers::pi / 10.0;
constexpr double FULL_TURN = 2.0 * std::numbers::pi;
} // namespace

int main()
{
    Common::initializeLogging();

    spdlog::info("Simple Delaunay2D Example - Points on a Circle");

    // Define the nodes on a circle
    std::vector<Point2D> points;

    for (double angle = 0.0; angle < FULL_TURN; angle += ANGLE_STEP)
    {
        points.emplace_back(std::cos(angle), std::sin(angle));
    }

    spdlog::info("Created {} points on the unit circle", points.size());
    for (std::size_t pointIndex = 0; pointIndex < points.size(); ++pointIndex)
    {
        spdlog::info("  Point {}: ({:.2f}, {:.2f})", pointIndex, points[pointIndex].x(), points[pointIndex].y());
    }

    MeshData2D meshData2D;

    // Perform triangulation
    spdlog::info("Running triangulation...");
    Delaunay2D::triangulate(points, meshData2D);

    // Export to VTK
    Export::VtkExporter exporter;
    exporter.writeVtu(meshData2D, "simple_delaunay_2d.vtu");
    Export::TsvExporter::writeMesh(meshData2D, "simple_delaunay_2d");

    spdlog::info("");
    spdlog::info("Mesh exported to simple_delaunay_2d.vtu and simple_delaunay_2d.{nodes,cells}.tsv");
    spdlog::info("Open in ParaView to visualize:");
    spdlog::info("  paraview simple_delaunay_2d.vtu");

    return 0;
}
