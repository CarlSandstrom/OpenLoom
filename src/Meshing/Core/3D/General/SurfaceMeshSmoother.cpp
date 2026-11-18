#include "Meshing/Core/3D/General/SurfaceMeshSmoother.h"

#include "Common/Types.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/SurfaceProjector.h"
#include "Meshing/Core/3D/General/TetrahedronInversionGuard.h"
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

std::unordered_map<NodeId, std::unordered_set<NodeId>> buildAdjacency(const SurfaceMesh3D& mesh)
{
    std::unordered_map<NodeId, std::unordered_set<NodeId>> adjacency;
    for (const auto& triangle : mesh.triangles)
    {
        for (size_t i = 0; i < 3; ++i)
        {
            auto node0 = triangle[i];
            auto node1 = triangle[(i + 1) % 3];
            adjacency[node0].insert(node1);
            adjacency[node1].insert(node0);
        }
    }
    return adjacency;
}

std::unordered_set<NodeId> collectFixedNodeIds(const SurfaceMesh3D& mesh)
{
    std::unordered_set<NodeId> fixed;
    for (const auto& [edgeId, nodeIds] : mesh.edgeNodeIds)
        for (auto nodeId : nodeIds)
            fixed.insert(nodeId);
    return fixed;
}

std::unordered_map<NodeId, std::string> buildNodeToFaceMap(const SurfaceMesh3D& mesh)
{
    std::unordered_map<NodeId, std::string> nodeToFace;
    for (const auto& [faceId, triangleIndices] : mesh.faceTriangleIds)
        for (auto triangleIndex : triangleIndices)
            for (auto nodeId : mesh.triangles[triangleIndex])
                nodeToFace.emplace(nodeId, faceId);
    return nodeToFace;
}

/// What a sweep needs to know about the mesh that moving nodes does not
/// change: who neighbors whom, which nodes are pinned to a CAD curve, and
/// which CAD surface each node is re-projected onto. Built once, read by
/// every sweep.
struct SurfaceNeighborhood
{
    std::unordered_map<NodeId, std::unordered_set<NodeId>> adjacency;
    std::unordered_set<NodeId> fixedNodeIds;
    std::unordered_map<NodeId, std::string> nodeToFace;
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
                                              const SurfaceNeighborhood& neighborhood)
{
    auto proposed = mesh.nodes;

    for (const auto& [nodeId, neighbors] : neighborhood.adjacency)
    {
        if (neighborhood.fixedNodeIds.count(nodeId) || neighbors.empty())
            continue;

        auto faceEntry = neighborhood.nodeToFace.find(nodeId);
        if (faceEntry == neighborhood.nodeToFace.end())
            continue;

        auto surface = geometry.getSurface(faceEntry->second);
        if (!surface)
            continue;

        Point3D centroid = Point3D::Zero();
        for (auto neighborId : neighbors)
            centroid += mesh.nodes[neighborId.value()];
        centroid /= static_cast<double>(neighbors.size());

        auto projected = SurfaceProjector::projectToSurface(centroid, *surface);
        if (projected)
            proposed[nodeId.value()] = *projected;
    }

    return proposed;
}

} // namespace

void SurfaceMeshSmoother::smooth(const Geometry3D::GeometryCollection3D& geometry,
                                 SurfaceMesh3D& mesh,
                                 std::size_t iterations,
                                 const std::vector<std::array<NodeId, 4>>& tetrahedra,
                                 double sliverAngleDegrees)
{
    if (mesh.nodes.empty() || iterations == 0)
        return;

    auto neighborhood = buildNeighborhood(mesh);
    TetrahedronInversionGuard inversionGuard(tetrahedra, sliverAngleDegrees);

    // Each iteration is a proposal the guard then vetoes moves out of: the
    // surface half decides where nodes want to go, the volume half decides
    // which of those moves the tetrahedra can live with.
    for (size_t iteration = 0; iteration < iterations; ++iteration)
    {
        auto proposed = proposeSmoothedPositions(geometry, mesh, neighborhood);
        inversionGuard.revertInvertingMoves(mesh.nodes, proposed);
        mesh.nodes = std::move(proposed);
    }
}

} // namespace Meshing
