#include "Meshing/Core/3D/RCDT/RCDTMeshExtractor.h"

#include "Meshing/Core/3D/RCDT/RestrictedFaceTypes.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

#include <algorithm>

namespace Meshing
{

namespace
{

void appendRestrictedTriangles(const RestrictedFaceMap& restrictedFaces,
                               std::vector<std::array<NodeId, 3>>& triangles,
                               std::map<std::string, std::vector<size_t>>& triangleIdsBySurface)
{
    for (const auto& [faceKey, surfaceId] : restrictedFaces)
    {
        triangleIdsBySurface[surfaceId].push_back(triangles.size());
        triangles.push_back({faceKey.nodeIds[0], faceKey.nodeIds[1], faceKey.nodeIds[2]});
    }
}

std::map<std::string, std::vector<NodeId>> buildEdgeNodeIds(const CurveSegmentManager& curveSegmentManager,
                                                            const Topology3D::Topology3D& topology)
{
    std::map<std::string, std::vector<NodeId>> edgeNodeIds;
    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        auto nodeIds = curveSegmentManager.getOrderedNodeIdsForEdge(edgeId);
        if (!nodeIds.empty())
            edgeNodeIds[edgeId] = std::move(nodeIds);
    }
    return edgeNodeIds;
}

} // namespace

// Zero-fill rather than plain resize(): node IDs below the highest surviving
// one may be gaps left by the removed supertet corners, and Eigen's default
// constructor does not zero-initialize — an unfilled slot would otherwise
// hold whatever was previously in that memory.
std::vector<Point3D> RCDTMeshExtractor::extractNodePositions(const MeshData3D& meshData)
{
    std::vector<Point3D> nodes;
    if (meshData.getNodeCount() == 0)
        return nodes;

    NodeId maxNodeId{0};
    for (const auto& [nodeId, node] : meshData.getNodes())
        maxNodeId = std::max(maxNodeId, nodeId);

    nodes.resize(maxNodeId.value() + 1, Point3D::Zero());
    for (const auto& [nodeId, node] : meshData.getNodes())
        nodes[nodeId.value()] = node->getCoordinates();
    return nodes;
}

SurfaceMesh3D RCDTMeshExtractor::extractSurfaceMesh(const MeshData3D& meshData,
                                                    const RestrictedFaceMap& restrictedFaces,
                                                    const Topology3D::Topology3D& topology)
{
    SurfaceMesh3D surfaceMesh;
    surfaceMesh.nodes = extractNodePositions(meshData);
    appendRestrictedTriangles(restrictedFaces, surfaceMesh.triangles, surfaceMesh.faceTriangleIds);
    surfaceMesh.edgeNodeIds = buildEdgeNodeIds(meshData.getCurveSegmentManager(), topology);

    spdlog::debug("RCDTMeshExtractor::extractSurfaceMesh: {} nodes, {} triangles, {} faces, {} edges",
                  surfaceMesh.nodes.size(), surfaceMesh.triangles.size(),
                  surfaceMesh.faceTriangleIds.size(), surfaceMesh.edgeNodeIds.size());

    return surfaceMesh;
}

VolumeMesh3D RCDTMeshExtractor::extractVolumeMesh(const MeshData3D& meshData,
                                                  const RestrictedFaceMap& restrictedFaces,
                                                  const Topology3D::Topology3D& topology)
{
    VolumeMesh3D volumeMesh;
    volumeMesh.nodes = extractNodePositions(meshData);
    volumeMesh.tetrahedra = extractTetrahedra(meshData);

    appendRestrictedTriangles(restrictedFaces, volumeMesh.boundaryTriangles,
                              volumeMesh.boundaryFaceTriangleIds);
    volumeMesh.boundaryEdgeNodeIds = buildEdgeNodeIds(meshData.getCurveSegmentManager(), topology);

    spdlog::debug("RCDTMeshExtractor::extractVolumeMesh: {} nodes, {} tetrahedra, {} boundary triangles, "
                  "{} boundary faces, {} boundary edges",
                  volumeMesh.nodes.size(), volumeMesh.tetrahedra.size(), volumeMesh.boundaryTriangles.size(),
                  volumeMesh.boundaryFaceTriangleIds.size(), volumeMesh.boundaryEdgeNodeIds.size());

    return volumeMesh;
}

std::vector<std::array<NodeId, 4>> RCDTMeshExtractor::extractTetrahedra(const MeshData3D& meshData)
{
    std::vector<std::array<NodeId, 4>> tetrahedra;
    for (const auto& [elementId, element] : meshData.getElements())
    {
        auto tetrahedron = dynamic_cast<const TetrahedronElement*>(element.get());
        if (!tetrahedron) continue;

        auto& nodeIds = tetrahedron->getNodeIds();
        tetrahedra.push_back({nodeIds[0], nodeIds[1], nodeIds[2], nodeIds[3]});
    }
    return tetrahedra;
}

} // namespace Meshing
