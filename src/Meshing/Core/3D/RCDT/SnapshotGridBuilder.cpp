#include "Meshing/Core/3D/RCDT/SnapshotGridBuilder.h"

#include "Export/VtkGrid.h"
#include "Meshing/Core/3D/RCDT/RCDTMeshExtractor.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Topology/Topology3D.h"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <unordered_map>
#include <vector>

namespace Meshing
{

namespace
{
std::vector<std::size_t> sequence(std::size_t count);
std::unordered_map<std::string, int> indexSurfaces(const Topology3D::Topology3D& topology);
// std::map orders the surface ids, so an index means the same surface across runs.
std::vector<int> triangleSurfaceIndices(const SurfaceMesh3D& surfaceMesh);
} // namespace

Export::VtkGrid SnapshotGridBuilder::fromSurfaceMesh(const SurfaceMesh3D& surfaceMesh)
{
    Export::VtkGrid grid;
    grid.points = surfaceMesh.nodes;
    grid.nodeIds = sequence(surfaceMesh.nodes.size());

    for (auto& triangle : surfaceMesh.triangles)
        grid.cells.push_back({Export::VtkCellType::Triangle, {triangle[0], triangle[1], triangle[2]}});
    grid.cellFields.push_back({"SurfaceID", triangleSurfaceIndices(surfaceMesh)});
    return grid;
}

Export::VtkGrid SnapshotGridBuilder::fromRestrictedFacets(
    const MeshData3D& meshData,
    const Topology3D::Topology3D& topology,
    const std::unordered_map<FaceKey, RestrictedFacet, FaceKeyHash>& facets,
    const std::unordered_map<FaceKey, SurfaceFacetCriteria::Badness, FaceKeyHash>& badness,
    const std::unordered_set<FaceKey, FaceKeyHash>& droppedFaces)
{
    Export::VtkGrid grid;
    grid.points = RCDTMeshExtractor::extractNodePositions(meshData);
    grid.nodeIds = sequence(grid.points.size());

    std::vector<FaceKey> faces;
    for (auto& [face, facet] : facets)
        faces.push_back(face);
    std::sort(faces.begin(), faces.end());

    auto surfaceIndices = indexSurfaces(topology);
    std::vector<int> surfaceIds;
    std::vector<int> criteria;
    std::vector<double> values;
    std::vector<int> dropped;
    for (auto& face : faces)
    {
        grid.cells.push_back({Export::VtkCellType::Triangle, {face.nodeIds[0], face.nodeIds[1], face.nodeIds[2]}});
        surfaceIds.push_back(surfaceIndices.at(facets.at(face).surfaceId));

        auto found = badness.find(face);
        criteria.push_back(found != badness.end() ? found->second.first : -1);
        values.push_back(found != badness.end() ? found->second.second : 0.0);
        dropped.push_back(droppedFaces.contains(face) ? 1 : 0);
    }
    grid.cellFields.push_back({"SurfaceID", std::move(surfaceIds)});
    grid.cellFields.push_back({"BadnessCriterion", std::move(criteria)});
    grid.cellFields.push_back({"BadnessValue", std::move(values)});
    grid.cellFields.push_back({"Dropped", std::move(dropped)});
    return grid;
}

Export::VtkGrid SnapshotGridBuilder::fromProtectingBalls(const MeshData3D& meshData)
{
    std::vector<NodeId> nodeIds;
    for (auto& [nodeId, node] : meshData.getNodes())
    {
        if (node->getWeight() > 0.0) nodeIds.push_back(nodeId);
    }
    std::sort(nodeIds.begin(), nodeIds.end());

    Export::VtkGrid grid;
    std::vector<double> radii;
    std::vector<double> weights;
    for (auto nodeId : nodeIds)
    {
        auto node = meshData.getNode(nodeId);
        grid.cells.push_back({Export::VtkCellType::Vertex, {grid.points.size()}});
        grid.points.push_back(node->getCoordinates());
        radii.push_back(std::sqrt(node->getWeight()));
        weights.push_back(node->getWeight());
    }
    grid.nodeIds = std::move(nodeIds);
    grid.pointFields.push_back({"Radius", std::move(radii)});
    grid.pointFields.push_back({"Weight", std::move(weights)});
    return grid;
}

namespace
{

std::vector<std::size_t> sequence(std::size_t count)
{
    std::vector<std::size_t> indices(count);
    std::iota(indices.begin(), indices.end(), std::size_t{0});
    return indices;
}

std::unordered_map<std::string, int> indexSurfaces(const Topology3D::Topology3D& topology)
{
    auto surfaceIds = topology.getAllSurfaceIds();
    std::sort(surfaceIds.begin(), surfaceIds.end());

    std::unordered_map<std::string, int> indices;
    for (auto& surfaceId : surfaceIds)
        indices.emplace(surfaceId, static_cast<int>(indices.size()));
    return indices;
}

std::vector<int> triangleSurfaceIndices(const SurfaceMesh3D& surfaceMesh)
{
    std::vector<int> indices(surfaceMesh.triangles.size(), -1);
    int surfaceIndex = 0;
    for (auto& [surfaceId, triangleIds] : surfaceMesh.faceTriangleIds)
    {
        for (auto triangleId : triangleIds)
            indices[triangleId] = surfaceIndex;
        ++surfaceIndex;
    }
    return indices;
}

} // namespace

} // namespace Meshing
