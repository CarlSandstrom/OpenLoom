#include "Meshing/Core/3D/Volume/Delaunay3D.h"
#include "Common/Exceptions/MeshException.h"
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

    // Left in the mesh -- see class documentation.
    operations.createBoundingTetrahedron(points);

    for (size_t i = 0; i < points.size(); ++i)
    {
        auto weightEntry = pointWeights.find(i);
        auto weight = weightEntry != pointWeights.end() ? weightEntry->second : 0.0;
        auto nodeId = operations.insertVertexBowyerWatson(points[i], i < geometryIds.size() ? geometryIds[i] : std::vector<std::string>{}, weight);

        // Every point lies inside the bounding tetrahedron, so it fails only when
        // a heavier vertex hides it -- and every seed point (a protecting ball or
        // a boundary sample) must become a vertex.
        if (!nodeId)
        {
            OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                                OpenLoom::MeshException::ErrorCode::GENERATION_FAILED,
                                "Delaunay3D::triangulate: point " + std::to_string(i) + " (weight " +
                                    std::to_string(weight) + ") is hidden by another vertex");
        }
        result.pointIndexToNodeIdMap[i] = *nodeId;

        if ((i + 1) % 100 == 0)
        {
            spdlog::debug("Delaunay3D::triangulate: Inserted {}/{} points", i + 1, points.size());
        }
    }

    spdlog::info("Delaunay3D::triangulate: Complete - {} points inserted", result.pointIndexToNodeIdMap.size());

    return result;
}

} // namespace Meshing
