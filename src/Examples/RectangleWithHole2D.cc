#include "Export/VtkExporter.h"
#include "Geometry2D/GeometryCollection2D.h"
#include "Geometry2D/LinearEdge2D.h"
#include "Meshing/Core/ConstrainedDelaunay2D.h"
#include "Meshing/Core/MeshingContext2D.h"
#include "Meshing/Data/MeshData2D.h"
#include "Meshing/Data/MeshData3D.h"
#include "Meshing/Data/MeshMutator3D.h"
#include "Meshing/Data/Node2D.h"
#include "Meshing/Data/TriangleElement.h"
#include "Topology2D/Topology2D.h"
#include "spdlog/spdlog.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

using namespace Meshing;

/**
 * @brief Convert MeshData2D to MeshData3D (3D with z=0) for VTK export
 */
MeshData3D convertToMeshData3D(const MeshData2D& meshData2D)
{
    MeshData3D meshData3D;
    MeshMutator3D operations(meshData3D);

    // Create a mapping from old node IDs to new node IDs
    std::unordered_map<size_t, size_t> nodeIdMap;

    // Convert nodes from 2D to 3D (set z=0)
    for (const auto& [nodeId, node2D] : meshData2D.getNodes())
    {
        const Point2D& point2D = node2D->getCoordinates();
        Point3D point3D(point2D.x(), point2D.y(), 0.0);
        size_t newNodeId = operations.addNode(point3D);
        nodeIdMap[nodeId] = newNodeId;
    }

    // Copy elements (triangles) with remapped node IDs
    for (const auto& [elemId, element] : meshData2D.getElements())
    {
        const auto* tri = dynamic_cast<const TriangleElement*>(element.get());
        if (tri)
        {
            const auto& oldNodeIds = tri->getNodeIdArray();
            std::array<size_t, 3> newNodeIds = {
                nodeIdMap[oldNodeIds[0]],
                nodeIdMap[oldNodeIds[1]],
                nodeIdMap[oldNodeIds[2]]};
            operations.addElement(std::make_unique<TriangleElement>(newNodeIds));
        }
    }

    return meshData3D;
}

int main()
{
    spdlog::set_level(spdlog::level::info);
    spdlog::set_pattern("[%H:%M:%S] %^%v%$");

    spdlog::info("Creating 2D rectangle with hole example");

    // Create geometry collection
    auto geometry = std::make_unique<Geometry2D::GeometryCollection2D>();

    // Define outer rectangle corners (10x8 units)
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c0", Point2D(0.0, 0.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c1", Point2D(10.0, 0.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c2", Point2D(10.0, 8.0)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("outer_c3", Point2D(0.0, 8.0)));

    // Define inner rectangle corners (hole: 4x3 units, centered)
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("inner_c0", Point2D(3.0, 2.5)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("inner_c1", Point2D(7.0, 2.5)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("inner_c2", Point2D(7.0, 5.5)));
    geometry->addCorner(std::make_unique<Geometry2D::Corner2D>("inner_c3", Point2D(3.0, 5.5)));

    // Create outer rectangle edges
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("outer_e0", Point2D(0.0, 0.0), Point2D(10.0, 0.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("outer_e1", Point2D(10.0, 0.0), Point2D(10.0, 8.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("outer_e2", Point2D(10.0, 8.0), Point2D(0.0, 8.0)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("outer_e3", Point2D(0.0, 8.0), Point2D(0.0, 0.0)));

    // Create inner rectangle edges (hole)
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("inner_e0", Point2D(3.0, 2.5), Point2D(7.0, 2.5)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("inner_e1", Point2D(7.0, 2.5), Point2D(7.0, 5.5)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("inner_e2", Point2D(7.0, 5.5), Point2D(3.0, 5.5)));
    geometry->addEdge(std::make_unique<Geometry2D::LinearEdge2D>("inner_e3", Point2D(3.0, 5.5), Point2D(3.0, 2.5)));

    // Create topology
    std::unordered_map<std::string, Topology2D::Corner2D> topoCorners;
    topoCorners.emplace("outer_c0", Topology2D::Corner2D("outer_c0", {"outer_e0", "outer_e3"}));
    topoCorners.emplace("outer_c1", Topology2D::Corner2D("outer_c1", {"outer_e0", "outer_e1"}));
    topoCorners.emplace("outer_c2", Topology2D::Corner2D("outer_c2", {"outer_e1", "outer_e2"}));
    topoCorners.emplace("outer_c3", Topology2D::Corner2D("outer_c3", {"outer_e2", "outer_e3"}));
    topoCorners.emplace("inner_c0", Topology2D::Corner2D("inner_c0", {"inner_e0", "inner_e3"}));
    topoCorners.emplace("inner_c1", Topology2D::Corner2D("inner_c1", {"inner_e0", "inner_e1"}));
    topoCorners.emplace("inner_c2", Topology2D::Corner2D("inner_c2", {"inner_e1", "inner_e2"}));
    topoCorners.emplace("inner_c3", Topology2D::Corner2D("inner_c3", {"inner_e2", "inner_e3"}));

    std::unordered_map<std::string, Topology2D::Edge2D> topoEdges;
    // Outer edges (counter-clockwise)
    topoEdges.emplace("outer_e0", Topology2D::Edge2D("outer_e0", "outer_c0", "outer_c1"));
    topoEdges.emplace("outer_e1", Topology2D::Edge2D("outer_e1", "outer_c1", "outer_c2"));
    topoEdges.emplace("outer_e2", Topology2D::Edge2D("outer_e2", "outer_c2", "outer_c3"));
    topoEdges.emplace("outer_e3", Topology2D::Edge2D("outer_e3", "outer_c3", "outer_c0"));
    // Inner edges (clockwise - for hole)
    topoEdges.emplace("inner_e0", Topology2D::Edge2D("inner_e0", "inner_c0", "inner_c1"));
    topoEdges.emplace("inner_e1", Topology2D::Edge2D("inner_e1", "inner_c1", "inner_c2"));
    topoEdges.emplace("inner_e2", Topology2D::Edge2D("inner_e2", "inner_c2", "inner_c3"));
    topoEdges.emplace("inner_e3", Topology2D::Edge2D("inner_e3", "inner_c3", "inner_c0"));

    // Boundary edge loop: outer boundary followed by inner boundary (hole)
    std::vector<std::string> boundaryEdgeLoop = {
        "outer_e0", "outer_e1", "outer_e2", "outer_e3",
        "inner_e0", "inner_e1", "inner_e2", "inner_e3"};

    auto topology = std::make_unique<Topology2D::Topology2D>(topoCorners, topoEdges, boundaryEdgeLoop);

    // Create meshing context
    MeshingContext2D context(std::move(geometry), std::move(topology));

    // Create constrained Delaunay triangulator
    ConstrainedDelaunay2D mesher(context);

    // Generate constrained mesh with 15 samples per edge
    spdlog::info("Generating constrained Delaunay triangulation...");
    mesher.generateConstrained(15);

    // Get the 2D mesh data
    const MeshData2D& meshData2D = mesher.getMeshData2D();

    spdlog::info("Mesh generated:");
    spdlog::info("  Nodes: {}", meshData2D.getNodeCount());
    spdlog::info("  Elements: {}", meshData2D.getElementCount());

    // Convert to 3D mesh data for export
    MeshData3D meshData3D = convertToMeshData3D(meshData2D);

    // Export to VTK
    Export::VtkExporter exporter;
    exporter.writeVtu(meshData3D, "rectangle_with_hole_2d.vtu");

    spdlog::info("");
    spdlog::info("Mesh exported to rectangle_with_hole_2d.vtu");
    spdlog::info("Open in ParaView to visualize:");
    spdlog::info("  paraview rectangle_with_hole_2d.vtu");

    return 0;
}
