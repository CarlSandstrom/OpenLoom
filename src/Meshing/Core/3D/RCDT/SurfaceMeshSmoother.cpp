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

/// What a sweep needs to know about the mesh that moving nodes does not
/// change: who neighbors whom, which nodes are pinned to a CAD curve, and
/// which CAD surface each node is re-projected onto. Built once, read by
/// every sweep.
struct SurfaceNeighborhood
{
    std::unordered_map<size_t, std::unordered_set<size_t>> adjacency;
    std::unordered_set<size_t> fixedNodeIds;
    std::unordered_map<size_t, std::string> nodeToFace;
};

SurfaceNeighborhood buildNeighborhood(const SurfaceMesh3D& mesh)
{
    return SurfaceNeighborhood{buildAdjacency(mesh), collectFixedNodeIds(mesh), buildNodeToFaceMap(mesh)};
}

// One Laplacian sweep: every movable node goes to the centroid of its
// neighbors, re-projected onto its own surface. The result is a proposal
// rather than a move, because it is read off the positions the mesh had when
// the sweep started -- every node moves against the same input, and none of
// them against a neighbor that has already moved.
//
// A node stays where it is when it is pinned to a CAD curve, when no surface
// claims it, or when the projection fails.
std::vector<Point3D> proposeSmoothedPositions(const Geometry3D::GeometryCollection3D& geometry,
                                              const SurfaceMesh3D& mesh,
                                              const SurfaceNeighborhood& neighborhood,
                                              const SurfaceProjector& projector)
{
    std::vector<Point3D> proposed = mesh.nodes;

    for (const auto& [nodeId, neighbors] : neighborhood.adjacency)
    {
        if (neighborhood.fixedNodeIds.count(nodeId) || neighbors.empty())
            continue;

        const auto faceIt = neighborhood.nodeToFace.find(nodeId);
        if (faceIt == neighborhood.nodeToFace.end())
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
            proposed[nodeId] = *projected;
    }

    return proposed;
}

} // namespace

void SurfaceMeshSmoother::smooth(const Geometry3D::GeometryCollection3D& geometry,
                                 SurfaceMesh3D& mesh,
                                 std::size_t iterations,
                                 const std::vector<std::array<std::size_t, 4>>& tetrahedra)
{
    if (mesh.nodes.empty() || iterations == 0)
        return;

    const SurfaceNeighborhood neighborhood = buildNeighborhood(mesh);
    const TetrahedronInversionGuard inversionGuard(tetrahedra);
    const SurfaceProjector projector;

    // Each iteration is a proposal the guard then vetoes moves out of: the
    // surface half decides where nodes want to go, the volume half decides
    // which of those moves the tetrahedra can live with.
    for (size_t iteration = 0; iteration < iterations; ++iteration)
    {
        std::vector<Point3D> proposed = proposeSmoothedPositions(geometry, mesh, neighborhood, projector);
        inversionGuard.revertInvertingMoves(mesh.nodes, proposed);
        mesh.nodes = std::move(proposed);
    }
}

} // namespace Meshing
