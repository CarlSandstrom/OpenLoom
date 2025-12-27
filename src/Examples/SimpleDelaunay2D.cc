#include "Export/VtkExporter.h"
#include "Meshing/Core/2D/Delaunay2D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "spdlog/spdlog.h"

#include <vector>

using namespace Meshing;

int main()
{
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] %^%v%$");

    spdlog::info("Simple Delaunay2D Example - Rectangle Corners");

    // Define the nodes on a circle
    std::vector<Point2D> points;

    for (double alpha = 0; alpha < 2 * M_PI; alpha += M_PI / 10.0)
    {
        double x = std::cos(alpha);
        double y = std::sin(alpha);
        points.emplace_back(x, y);
    }

    spdlog::info("Created {} points forming a rectangle", points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
        spdlog::info("  Point {}: ({:.2f}, {:.2f})", i, points[i].x(), points[i].y());
    }

    // Create Delaunay triangulator
    MeshData2D meshData2D;
    Delaunay2D triangulator(points, &meshData2D);

    // Perform triangulation
    spdlog::info("Running triangulation...");
    triangulator.triangulate();
    const auto& triangles = meshData2D.getElements();
    const auto& nodes = meshData2D.getNodes();

    // Create 3D mesh data for export (with z=0)
    MeshData3D meshData3D(meshData2D);
    // Export to VTK
    Export::VtkExporter exporter;
    exporter.writeVtu(meshData3D, "simple_delaunay_2d.vtu");

    spdlog::info("");
    spdlog::info("Mesh exported to simple_delaunay_2d.vtu");
    spdlog::info("Open in ParaView to visualize:");
    spdlog::info("  paraview simple_delaunay_2d.vtu");

    return 0;
}
