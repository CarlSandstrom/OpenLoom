#include "Delaunay2D.h"
#include "GeometryUtilities2D.h"
#include "Meshing/Core/2D/MeshOperations2D.h"
#include "Meshing/Data/2D/MeshMutator2D.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

std::map<size_t, size_t> Delaunay2D::triangulate(const std::vector<Point2D>& points,
                                                 MeshData2D& meshData,
                                                 const std::vector<std::vector<std::string>>& geometryIds)
{
    MeshMutator2D meshMutator(meshData);
    MeshOperations2D meshOperations(meshData);

    spdlog::info("Delaunay2D::triangulate() called with {} points", points.size());
    auto [p0, p1, p2] = GeometryUtilities2D::createSuperTriangle(points);
    spdlog::info("Super triangle points: ({:.2f}, {:.2f}), ({:.2f}, {:.2f}), ({:.2f}, {:.2f})",
                 p0.x(), p0.y(), p1.x(), p1.y(), p2.x(), p2.y());
    size_t superNodeId0 = meshMutator.addNode(p0);
    size_t superNodeId1 = meshMutator.addNode(p1);
    size_t superNodeId2 = meshMutator.addNode(p2);

    auto superTriangle = std::make_unique<TriangleElement>(std::array<size_t, 3>{superNodeId0, superNodeId1, superNodeId2});
    size_t superElementId = meshMutator.addElement(std::move(superTriangle));

    std::map<size_t, size_t> pointIndexToNodeIdMap;
    size_t index = 0;
    for (const auto& point : points)
    {
        size_t nodeId;

        bool hasGeometryIds = index < geometryIds.size() && !geometryIds[index].empty();

        if (hasGeometryIds)
        {
            nodeId = meshOperations.insertVertexBowyerWatson(point, geometryIds[index]);
        }
        else
        {
            nodeId = meshOperations.insertVertexBowyerWatson(point);
        }

        pointIndexToNodeIdMap[index] = nodeId;
        ++index;
    }

    meshOperations.removeTrianglesContainingNode(superNodeId0);
    meshOperations.removeTrianglesContainingNode(superNodeId1);
    meshOperations.removeTrianglesContainingNode(superNodeId2);

    return pointIndexToNodeIdMap;
}

} // namespace Meshing
