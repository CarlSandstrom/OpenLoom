#include "MeshLogger.h"
#include <iostream>
#include <iomanip>

namespace Meshing
{

void MeshLogger::logMeshData2D(const MeshData2D& meshData)
{
    std::cout << "=== MeshData2D Summary ===" << std::endl;
    std::cout << "Total Nodes: " << meshData.getNodeCount() << std::endl;
    std::cout << "Total Elements: " << meshData.getElementCount() << std::endl;
    std::cout << std::endl;

    // Log nodes
    std::cout << "=== Nodes ===" << std::endl;
    const auto& nodes = meshData.getNodes();
    for (const auto& [id, node] : nodes)
    {
        const auto& coords = node->getCoordinates();
        std::cout << "Node " << std::setw(6) << id
                  << ": (" << std::setw(12) << coords.x()
                  << ", " << std::setw(12) << coords.y() << ")"
                  << (node->isBoundary() ? " [Boundary]" : "");

        const auto& geometryIds = node->getGeometryIds();
        if (!geometryIds.empty())
        {
            std::cout << " GeomIDs: [";
            for (size_t i = 0; i < geometryIds.size(); ++i)
            {
                std::cout << geometryIds[i];
                if (i < geometryIds.size() - 1)
                {
                    std::cout << ", ";
                }
            }
            std::cout << "]";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;

    // Log elements
    std::cout << "=== Elements ===" << std::endl;
    const auto& elements = meshData.getElements();
    for (const auto& [id, element] : elements)
    {
        std::cout << "Element " << std::setw(6) << id;

        // Log element type
        switch (element->getType())
        {
            case ElementType::TRIANGLE:
                std::cout << " [Triangle]";
                break;
            case ElementType::QUADRILATERAL:
                std::cout << " [Quadrilateral]";
                break;
            case ElementType::TETRAHEDRON:
                std::cout << " [Tetrahedron]";
                break;
            case ElementType::HEXAHEDRON:
                std::cout << " [Hexahedron]";
                break;
            case ElementType::PRISM:
                std::cout << " [Prism]";
                break;
            case ElementType::PYRAMID:
                std::cout << " [Pyramid]";
                break;
        }

        // Log node IDs
        std::cout << " Nodes: [";
        const auto& nodeIds = element->getNodeIds();
        for (size_t i = 0; i < nodeIds.size(); ++i)
        {
            std::cout << nodeIds[i];
            if (i < nodeIds.size() - 1)
            {
                std::cout << ", ";
            }
        }
        std::cout << "]" << std::endl;
    }
}

} // namespace Meshing
