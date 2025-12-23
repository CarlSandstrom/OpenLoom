#include "Delaunay2D.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

Delaunay2D::Delaunay2D(const std::vector<Point2D>& points, MeshData2D* meshData) :
    points_(points),
    meshData_(meshData),
    meshMutator_(*meshData_),
    meshOperations_(*meshData_),
    computer_(*meshData_)
{
}

void Delaunay2D::triangulate()
{
    spdlog::info("Delaunay2D::triangulate() called with {} points", points_.size());
    auto [p0, p1, p2] = computer_.createSuperTriangle(points_);
    spdlog::info("Super triangle points: ({:.2f}, {:.2f}), ({:.2f}, {:.2f}), ({:.2f}, {:.2f})",
                 p0.x(), p0.y(), p1.x(), p1.y(), p2.x(), p2.y());
    size_t superNodeId0 = meshMutator_.addNode(p0);
    size_t superNodeId1 = meshMutator_.addNode(p1);
    size_t superNodeId2 = meshMutator_.addNode(p2);

    auto superTriangle = std::make_unique<TriangleElement>(std::array<size_t, 3>{superNodeId0, superNodeId1, superNodeId2});
    size_t superElementId = meshMutator_.addElement(std::move(superTriangle));

    pointIndexToNodeIdMap_.clear();
    size_t index = 0;
    for (const auto& point : points_)
    {
        size_t nodeId = meshOperations_.insertVertexBowyerWatson(point);
        pointIndexToNodeIdMap_[index] = nodeId;
        ++index;
    }

    meshOperations_.removeTrianglesContainingNode(superNodeId0);
    meshOperations_.removeTrianglesContainingNode(superNodeId1);
    meshOperations_.removeTrianglesContainingNode(superNodeId2);
}

} // namespace Meshing
