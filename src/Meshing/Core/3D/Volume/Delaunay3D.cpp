#include "Meshing/Core/3D/Volume/Delaunay3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

Delaunay3DResult Delaunay3D::triangulate(MeshOperations3D& operations,
                                         const std::vector<Point3D>& points,
                                         const std::vector<std::vector<std::string>>& geometryIds,
                                         const std::unordered_map<size_t, double>& pointWeights)
{
    Delaunay3DResult result;

    if (points.empty())
    {
        spdlog::warn("Delaunay3D::triangulate: Empty point list");
        return result;
    }

    spdlog::info("Delaunay3D::triangulate: Starting with {} points", points.size());

    // Create bounding tetrahedron. Left in the mesh -- see class documentation.
    operations.createBoundingTetrahedron(points);

    // Insert each point using Bowyer-Watson
    for (size_t i = 0; i < points.size(); ++i)
    {
        bool hasGeomIds = i < geometryIds.size() && !geometryIds[i].empty();
        const auto weightIt = pointWeights.find(i);
        const double weight = weightIt != pointWeights.end() ? weightIt->second : 0.0;

        size_t nodeId;
        if (hasGeomIds)
        {
            nodeId = operations.insertVertexBowyerWatson(points[i], geometryIds[i], weight);
        }
        else
        {
            nodeId = operations.insertVertexBowyerWatson(points[i], {}, weight);
        }

        result.pointIndexToNodeIdMap[i] = nodeId;

        if ((i + 1) % 100 == 0)
        {
            spdlog::debug("Delaunay3D::triangulate: Inserted {}/{} points", i + 1, points.size());
        }
    }

    spdlog::info("Delaunay3D::triangulate: Complete - {} points inserted", result.pointIndexToNodeIdMap.size());

    return result;
}

} // namespace Meshing
