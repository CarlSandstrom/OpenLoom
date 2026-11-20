#include "Meshing/Core/3D/General/UnreferencedNodeRemover.h"

#include "Meshing/Data/3D/SurfaceMesh3D.h"
#include "Meshing/Data/3D/VolumeMesh3D.h"

#include <array>
#include <map>
#include <string>
#include <vector>

namespace Meshing
{

namespace
{

template <size_t N>
void markReferenced(const std::vector<std::array<NodeId, N>>& cells, std::vector<bool>& referenced);
void markReferenced(const std::map<std::string, std::vector<NodeId>>& edgeNodeIds, std::vector<bool>& referenced);
std::vector<NodeId> buildRenumbering(const std::vector<bool>& referenced);
void compactNodes(const std::vector<NodeId>& renumbering, std::vector<Point3D>& nodes);
template <size_t N>
void renumber(const std::vector<NodeId>& renumbering, std::vector<std::array<NodeId, N>>& cells);
void renumber(const std::vector<NodeId>& renumbering, std::map<std::string, std::vector<NodeId>>& edgeNodeIds);

} // namespace

std::vector<NodeId> UnreferencedNodeRemover::remove(SurfaceMesh3D& surfaceMesh)
{
    std::vector<bool> referenced(surfaceMesh.nodes.size(), false);
    markReferenced(surfaceMesh.triangles, referenced);
    markReferenced(surfaceMesh.edgeNodeIds, referenced);

    auto renumbering = buildRenumbering(referenced);
    compactNodes(renumbering, surfaceMesh.nodes);
    renumber(renumbering, surfaceMesh.triangles);
    renumber(renumbering, surfaceMesh.edgeNodeIds);
    return renumbering;
}

std::vector<NodeId> UnreferencedNodeRemover::remove(VolumeMesh3D& volumeMesh)
{
    std::vector<bool> referenced(volumeMesh.nodes.size(), false);
    markReferenced(volumeMesh.tetrahedra, referenced);
    markReferenced(volumeMesh.boundaryTriangles, referenced);
    markReferenced(volumeMesh.boundaryEdgeNodeIds, referenced);

    auto renumbering = buildRenumbering(referenced);
    compactNodes(renumbering, volumeMesh.nodes);
    renumber(renumbering, volumeMesh.tetrahedra);
    renumber(renumbering, volumeMesh.boundaryTriangles);
    renumber(renumbering, volumeMesh.boundaryEdgeNodeIds);
    return renumbering;
}

namespace
{

template <size_t N>
void markReferenced(const std::vector<std::array<NodeId, N>>& cells, std::vector<bool>& referenced)
{
    for (auto& cell : cells)
        for (auto nodeId : cell)
            referenced[nodeId.value()] = true;
}

void markReferenced(const std::map<std::string, std::vector<NodeId>>& edgeNodeIds, std::vector<bool>& referenced)
{
    for (auto& [edgeId, nodeIds] : edgeNodeIds)
        for (auto nodeId : nodeIds)
            referenced[nodeId.value()] = true;
}

// New ID of each old node, INVALID for a node that is dropped.
std::vector<NodeId> buildRenumbering(const std::vector<bool>& referenced)
{
    std::vector<NodeId> renumbering(referenced.size(), NodeId::INVALID);
    size_t nextId = 0;
    for (size_t oldId = 0; oldId < referenced.size(); ++oldId)
        if (referenced[oldId]) renumbering[oldId] = NodeId{nextId++};
    return renumbering;
}

void compactNodes(const std::vector<NodeId>& renumbering, std::vector<Point3D>& nodes)
{
    std::vector<Point3D> compacted;
    for (size_t oldId = 0; oldId < nodes.size(); ++oldId)
        if (renumbering[oldId] != NodeId::INVALID) compacted.push_back(nodes[oldId]);
    nodes = std::move(compacted);
}

template <size_t N>
void renumber(const std::vector<NodeId>& renumbering, std::vector<std::array<NodeId, N>>& cells)
{
    for (auto& cell : cells)
        for (auto& nodeId : cell)
            nodeId = renumbering[nodeId.value()];
}

void renumber(const std::vector<NodeId>& renumbering, std::map<std::string, std::vector<NodeId>>& edgeNodeIds)
{
    for (auto& [edgeId, nodeIds] : edgeNodeIds)
        for (auto& nodeId : nodeIds)
            nodeId = renumbering[nodeId.value()];
}

} // namespace

} // namespace Meshing
