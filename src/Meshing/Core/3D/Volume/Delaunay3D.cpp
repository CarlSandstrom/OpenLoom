#include "Meshing/Core/3D/Volume/Delaunay3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

Delaunay3D::Delaunay3D(MeshOperations3D& operations,
                       const std::vector<Point3D>& points,
                       const std::vector<std::vector<std::string>>& geometryIds,
                       const std::vector<double>& pointWeights) :
    operations_(operations),
    points_(points),
    geometryIds_(geometryIds),
    pointWeights_(pointWeights)
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

    // Create bounding tetrahedron. Left in the mesh -- see class documentation.
    boundingNodeIds_ = operations_.createBoundingTetrahedron(points_);

    // Insert each point using Bowyer-Watson
    pointIndexToNodeIdMap_.clear();

    for (size_t i = 0; i < points_.size(); ++i)
    {
        bool hasGeomIds = i < geometryIds_.size() && !geometryIds_[i].empty();
        const double weight = i < pointWeights_.size() ? pointWeights_[i] : 0.0;

        size_t nodeId;
        if (hasGeomIds)
        {
            nodeId = operations_.insertVertexBowyerWatson(points_[i], geometryIds_[i], weight);
        }
        else
        {
            nodeId = operations_.insertVertexBowyerWatson(points_[i], {}, weight);
        }

        pointIndexToNodeIdMap_[i] = nodeId;

        if ((i + 1) % 100 == 0)
        {
            spdlog::debug("Delaunay3D::triangulate: Inserted {}/{} points", i + 1, points_.size());
        }
    }

    spdlog::info("Delaunay3D::triangulate: Complete - {} points inserted", pointIndexToNodeIdMap_.size());
}

} // namespace Meshing
