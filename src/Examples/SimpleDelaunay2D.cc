#include "Export/VtkExporter.h"
#include "Meshing/Core/2D/Delaunay2D.h"
#include "Meshing/Data/MeshData3D.h"
#include "Meshing/Data/MeshMutator3D.h"
#include "Meshing/Data/TriangleElement.h"
#include "spdlog/spdlog.h"

#include <vector>

using namespace Meshing;

int main()
{
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] %^%v%$");

    spdlog::info("Simple Delaunay2D Example - Rectangle Corners");

    // Define the corners of a rectangle
    std::vector<Point2D> points = {
        Point2D(0.0, 0.0), // Bottom-left
        Point2D(5.0, 0.0), // Bottom-right
        Point2D(5.0, 3.0), // Top-right
        Point2D(0.0, 3.0)  // Top-left
    };

    spdlog::info("Created {} points forming a rectangle", points.size());
    for (size_t i = 0; i < points.size(); ++i)
    {
        spdlog::info("  Point {}: ({:.2f}, {:.2f})", i, points[i].x(), points[i].y());
    }

    // Create Delaunay triangulator
    Delaunay2D triangulator(points);

    // Perform triangulation
    spdlog::info("Running triangulation...");
    triangulator.triangulate();
    const auto& triangles = triangulator.getMeshData()->getElements();
    const auto& nodes = triangulator.getMeshData()->getNodes();

    // Create 3D mesh data for export (with z=0)
    MeshData3D meshData3D(*triangulator.getMeshData());

    // Export to VTK
    Export::VtkExporter exporter;
    exporter.writeVtu(meshData3D, "simple_delaunay_2d.vtu");

    spdlog::info("");
    spdlog::info("Mesh exported to simple_delaunay_2d.vtu");
    spdlog::info("Open in ParaView to visualize:");
    spdlog::info("  paraview simple_delaunay_2d.vtu");

    return 0;
}
