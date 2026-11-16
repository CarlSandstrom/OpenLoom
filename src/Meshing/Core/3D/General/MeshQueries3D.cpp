#include "Meshing/Core/3D/General/MeshQueries3D.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/General/RegularPredicates3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include <algorithm>
#include <array>
#include <map>

namespace Meshing
{

MeshQueries3D::MeshQueries3D(const MeshData3D& meshData) :
    meshData_(meshData)
{
}

std::vector<ElementId> MeshQueries3D::findConflictingTetrahedra(const Point3D& point, double pointWeight) const
{
    std::vector<ElementId> conflicting;

    for (const auto& [tetId, element] : meshData_.getElements())
    {
        auto tet = dynamic_cast<const TetrahedronElement*>(element.get());
        if (!tet) continue;

        // Weighted in-sphere ("orthosphere") test via a robust exact-
        // arithmetic determinant sign, not an explicit circumcenter/radius:
        // for a near-degenerate (nearly flat) tetrahedron, solving for the
        // circumcenter is a near-singular linear system whose solution can
        // be wrong by orders of magnitude, which silently corrupts this
        // conflict search (see RobustPredicates3D, OPE-159). Reduces
        // exactly to the plain (unweighted) test when every weight involved
        // is 0 (see RegularPredicates3D, OPE-176).
        const auto& nodeIds = tet->getNodeIds();
        auto node0 = meshData_.getNode(nodeIds[0]);
        auto node1 = meshData_.getNode(nodeIds[1]);
        auto node2 = meshData_.getNode(nodeIds[2]);
        auto node3 = meshData_.getNode(nodeIds[3]);
        if (!node0 || !node1 || !node2 || !node3) continue;

        if (RegularPredicates3D::insidePointOrthosphere(
                node0->getCoordinates(), node0->getWeight(),
                node1->getCoordinates(), node1->getWeight(),
                node2->getCoordinates(), node2->getWeight(),
                node3->getCoordinates(), node3->getWeight(),
                point, pointWeight))
        {
            conflicting.push_back(tetId);
        }
    }

    return conflicting;
}

std::vector<std::array<NodeId, 3>>
MeshQueries3D::findCavityBoundary(const std::vector<ElementId>& conflictingIndices) const
{
    std::map<FaceKey, size_t> faceCount;
    std::map<FaceKey, std::array<NodeId, 3>> faceOrientation;

    for (ElementId tetId : conflictingIndices)
    {
        auto element = meshData_.getElement(tetId);
        auto tet = dynamic_cast<const TetrahedronElement*>(element);
        if (!tet) continue;

        for (auto& face : tet->getFaces())
        {
            FaceKey key(face[0], face[1], face[2]);
            faceCount[key]++;
            // First occurrence wins; a boundary face occurs only once
            faceOrientation.try_emplace(key, face);
        }
    }

    // A boundary face is not shared with another conflicting tetrahedron
    std::vector<std::array<NodeId, 3>> boundary;
    for (const auto& [faceKey, count] : faceCount)
    {
        if (count == 1)
        {
            boundary.push_back(faceOrientation[faceKey]);
        }
    }

    return boundary;
}

std::vector<ElementId> MeshQueries3D::findTetrahedraWithFace(NodeId nodeId1, NodeId nodeId2, NodeId nodeId3) const
{
    std::vector<ElementId> result;

    for (const auto& [tetId, element] : meshData_.getElements())
    {
        auto tet = dynamic_cast<const TetrahedronElement*>(element.get());
        if (!tet) continue;

        auto& nodes = tet->getNodeIds();
        auto hasNode = [&nodes](NodeId nodeId)
        { return std::ranges::find(nodes, nodeId) != nodes.end(); };
        if (hasNode(nodeId1) && hasNode(nodeId2) && hasNode(nodeId3))
        {
            result.push_back(tetId);
        }
    }
    return result;
}

std::optional<NodeId> MeshQueries3D::findOppositeVertex(ElementId tetId, NodeId faceNode1, NodeId faceNode2, NodeId faceNode3) const
{
    auto element = meshData_.getElement(tetId);
    auto tet = dynamic_cast<const TetrahedronElement*>(element);
    if (!tet) return std::nullopt;

    for (NodeId nodeId : tet->getNodeIds())
    {
        if (nodeId != faceNode1 && nodeId != faceNode2 && nodeId != faceNode3) return nodeId;
    }
    return std::nullopt;
}

} // namespace Meshing
