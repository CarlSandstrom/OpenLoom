/**
 * @file export_example.h
 * @brief Example showing how to use the separated mesh data classes for export
 *
 * This example demonstrates the benefit of separation of concerns:
 * - Export only needs MeshData3D (lightweight data access)
 * - No heavy connectivity maps or operation infrastructure needed
 */

#pragma once
#include "Common/Types.h"
#include "MeshData3D.h"
#include "Meshing/Data/MeshConnectivity.h"
#include "Meshing/Data/MeshMutator3D.h"
#include <fstream>
#include <iostream>

namespace Examples
{

/**
 * @brief Simple mesh exporter that only needs the pure data container
 *
 * This example shows how export operations can now work with just
 * the lightweight MeshData3D class, without carrying the overhead
 * of operations and connectivity.
 */
class SimpleMeshExporter
{
public:
    explicit SimpleMeshExporter(const Meshing::MeshData3D& geometry) :
        geometry_(geometry) {}

    void exportToVTK(const std::string& filename) const
    {
        std::ofstream file(filename);

        // Write VTK header
        file << "# vtk DataFile Version 3.0\n";
        file << "Mesh Data\n";
        file << "ASCII\n";
        file << "DATASET UNSTRUCTURED_GRID\n";

        // Export nodes (vertices)
        const auto& nodes = geometry_.getNodes();
        file << "POINTS " << nodes.size() << " float\n";

        for (const auto& [id, node] : nodes)
        {
            const auto& coords = node->getCoordinates();
            file << coords.x() << " " << coords.y() << " " << coords.z() << "\n";
        }

        // Export elements (cells)
        const auto& elements = geometry_.getElements();
        size_t totalConnectivity = 0;
        for (const auto& [id, element] : elements)
        {
            totalConnectivity += element->getNodeCount() + 1; // +1 for cell size
        }

        file << "CELLS " << elements.size() << " " << totalConnectivity << "\n";

        for (const auto& [id, element] : elements)
        {
            const auto& nodeIds = element->getNodeIds();
            file << nodeIds.size();
            for (size_t nodeId : nodeIds)
            {
                file << " " << nodeId;
            }
            file << "\n";
        }

        // Export cell types
        file << "CELL_TYPES " << elements.size() << "\n";
        for (const auto& [id, element] : elements)
        {
            // VTK cell type for tetrahedron is 10
            file << "10\n";
        }
    }

private:
    const Meshing::MeshData3D& geometry_;
};

/**
 * @brief Example usage of the separated mesh data classes
 */
inline void demonstrateSeparatedDesign()
{
    using namespace Meshing;

    // 1. Create the separated components
    MeshData3D geometry;                              // Pure data storage
    Meshing::MeshMutator3D operations(geometry);      // Operations on data
    Meshing::MeshConnectivity connectivity(geometry); // Connectivity queries

    // Wire up operations to use connectivity for validation
    operations.setConnectivity(&connectivity);

    // 2. Build some mesh data using operations
    size_t n1 = operations.addNode(Point3D(0.0, 0.0, 0.0));
    size_t n2 = operations.addNode(Point3D(1.0, 0.0, 0.0));
    size_t n3 = operations.addNode(Point3D(0.0, 1.0, 0.0));
    size_t n4 = operations.addNode(Point3D(0.0, 0.0, 1.0));

    // Rebuild connectivity after adding nodes
    connectivity.rebuildConnectivity();

    std::cout << "Created mesh with " << geometry.getNodeCount() << " nodes\n";

    // 3. Export using only the lightweight geometry container
    SimpleMeshExporter exporter(geometry);
    exporter.exportToVTK("example_mesh.vtk");

    std::cout << "Exported mesh - exporter only needed MeshData3D!\n";
    std::cout << "No heavy connectivity maps or operations overhead in export.\n";

    // 4. Demonstrate connectivity queries (when needed)
    std::cout << "Node 1 is connected to "
              << connectivity.getNodeElements(n1).size() << " elements\n";
}

} // namespace Examples