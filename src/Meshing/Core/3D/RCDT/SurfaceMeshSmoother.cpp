#include "Meshing/Core/3D/RCDT/SurfaceMeshSmoother.h"

#include "Common/Types.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"
#include "Meshing/Core/3D/RCDT/TetrahedronInversionGuard.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"

#include <array>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace Meshing
{

namespace
{

std::unordered_map<size_t, std::unordered_set<size_t>> buildAdjacency(const SurfaceMesh3D& mesh)
{
    std::unordered_map<size_t, std::unordered_set<size_t>> adjacency;
    for (const auto& triangle : mesh.triangles)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            const size_t a = triangle[i];
            const size_t b = triangle[(i + 1) % 3];
            adjacency[a].insert(b);
            adjacency[b].insert(a);
        }
    }
    return adjacency;
}

std::unordered_set<size_t> collectFixedNodeIds(const SurfaceMesh3D& mesh)
{
    std::unordered_set<size_t> fixed;
    for (const auto& [edgeId, nodeIds] : mesh.edgeNodeIds)
        for (const size_t nodeId : nodeIds)
            fixed.insert(nodeId);
    return fixed;
}

std::unordered_map<size_t, std::string> buildNodeToFaceMap(const SurfaceMesh3D& mesh)
{
    std::unordered_map<size_t, std::string> nodeToFace;
    for (const auto& [faceId, triangleIndices] : mesh.faceTriangleIds)
        for (const size_t triangleIndex : triangleIndices)
            for (const size_t nodeId : mesh.triangles[triangleIndex])
                nodeToFace.emplace(nodeId, faceId);
    return nodeToFace;
}

} // namespace

void SurfaceMeshSmoother::smooth(const Geometry3D::GeometryCollection3D& geometry,
                                 SurfaceMesh3D& mesh,
                                 std::size_t iterations,
                                 const std::vector<std::array<std::size_t, 4>>& tetrahedra)
{
    if (mesh.nodes.empty() || iterations == 0)
        return;

    const auto adjacency = buildAdjacency(mesh);
    const auto fixedNodeIds = collectFixedNodeIds(mesh);
    const auto nodeToFace = buildNodeToFaceMap(mesh);

    const TetrahedronInversionGuard inversionGuard(tetrahedra);
    const SurfaceProjector projector;

    for (size_t iteration = 0; iteration < iterations; ++iteration)
    {
        std::vector<Point3D> updated = mesh.nodes;

        for (const auto& [nodeId, neighbors] : adjacency)
        {
            if (fixedNodeIds.count(nodeId) || neighbors.empty())
                continue;

            const auto faceIt = nodeToFace.find(nodeId);
            if (faceIt == nodeToFace.end())
                continue;

            const Geometry3D::ISurface3D* surface = geometry.getSurface(faceIt->second);
            if (!surface)
                continue;

            Point3D centroid = Point3D::Zero();
            for (const size_t neighborId : neighbors)
                centroid += mesh.nodes[neighborId];
            centroid /= static_cast<double>(neighbors.size());

            const auto projected = projector.projectToSurface(centroid, *surface);
            if (projected)
                updated[nodeId] = *projected;
        }

        inversionGuard.revertInvertingMoves(mesh.nodes, updated);
        mesh.nodes = std::move(updated);
    }
}

} // namespace Meshing
