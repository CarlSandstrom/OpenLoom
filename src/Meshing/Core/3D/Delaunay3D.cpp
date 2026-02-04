#include "Delaunay3D.h"
#include "MeshOperations3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

Delaunay3D::Delaunay3D(const std::vector<Point3D>& points,
                       MeshData3D* meshData,
                       const std::vector<std::vector<double>>& edgeParameters,
                       const std::vector<std::vector<std::string>>& geometryIds) :
    meshData_(meshData),
    points_(points),
    edgeParameters_(edgeParameters),
    geometryIds_(geometryIds)
{
}

void Delaunay3D::triangulate()
{
    if (points_.empty())
    {
        spdlog::warn("Delaunay3D::triangulate: Empty point list");
        return;
    }

    spdlog::info("Delaunay3D::triangulate: Starting with {} points", points_.size());

    MeshOperations3D operations(*meshData_);

    // Create bounding tetrahedron
    auto boundingIds = operations.createBoundingTetrahedron(points_);

    // Insert each point using Bowyer-Watson
    pointIndexToNodeIdMap_.clear();

    for (size_t i = 0; i < points_.size(); ++i)
    {
        // Check if this point has edge parameters and geometry IDs
        bool hasEdgeParams = i < edgeParameters_.size() && !edgeParameters_[i].empty();
        bool hasGeomIds = i < geometryIds_.size() && !geometryIds_[i].empty();

        size_t nodeId;
        if (hasEdgeParams && hasGeomIds)
        {
            nodeId = operations.insertVertexBowyerWatson(points_[i],
                                                          edgeParameters_[i],
                                                          geometryIds_[i]);
        }
        else if (hasGeomIds)
        {
            nodeId = operations.insertVertexBowyerWatson(points_[i], {}, geometryIds_[i]);
        }
        else
        {
            nodeId = operations.insertVertexBowyerWatson(points_[i]);
        }

        pointIndexToNodeIdMap_[i] = nodeId;

        if ((i + 1) % 100 == 0)
        {
            spdlog::debug("Delaunay3D::triangulate: Inserted {}/{} points", i + 1, points_.size());
        }
    }

    // Remove bounding tetrahedron
    operations.removeBoundingTetrahedron(boundingIds);

    spdlog::info("Delaunay3D::triangulate: Complete - {} nodes, {} tetrahedra",
                 meshData_->getNodeCount(), meshData_->getElementCount());
}

} // namespace Meshing
