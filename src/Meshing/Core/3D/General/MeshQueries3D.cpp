#include "Meshing/Core/3D/General/MeshQueries3D.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/General/RegularPredicates3D.h"
#include "Meshing/Core/3D/General/RobustPredicates3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include <algorithm>
#include <array>
#include <map>

namespace Meshing
{

namespace
{

std::vector<ElementId> connectedToPoint(const std::vector<ElementId>& conflicting, const Point3D& point, const MeshData3D& meshData);
bool containsPoint(const TetrahedronElement& tetrahedron, const Point3D& point, const MeshData3D& meshData);

} // namespace

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

    return connectedToPoint(conflicting, point, meshData_);
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

namespace
{

/// The conflicting tetrahedra reachable through shared faces from one that
/// contains point, in their original order. In a regular triangulation the
/// conflict region is connected, so this drops nothing; but a tetrahedron
/// whose orthosphere passes exactly through point counts as conflicting, and
/// equal weights on cocircular points make such ties common far from point.
/// Left in, a stray piece pinches the cavity along an edge and fanning it
/// from point stacks tetrahedra on one face (OPE-251).
std::vector<ElementId> connectedToPoint(const std::vector<ElementId>& conflicting, const Point3D& point, const MeshData3D& meshData)
{
    std::map<FaceKey, std::vector<size_t>> tetrahedraOfFace;
    std::vector<size_t> pending;
    std::vector<bool> reached(conflicting.size(), false);
    for (size_t index = 0; index < conflicting.size(); ++index)
    {
        auto& tetrahedron = dynamic_cast<const TetrahedronElement&>(*meshData.getElement(conflicting[index]));
        for (auto& face : tetrahedron.getFaces())
            tetrahedraOfFace[FaceKey(face[0], face[1], face[2])].push_back(index);
        if (!containsPoint(tetrahedron, point, meshData)) continue;
        reached[index] = true;
        pending.push_back(index);
    }

    while (!pending.empty())
    {
        auto index = pending.back();
        pending.pop_back();
        auto& tetrahedron = dynamic_cast<const TetrahedronElement&>(*meshData.getElement(conflicting[index]));
        for (auto& face : tetrahedron.getFaces())
            for (auto neighbour : tetrahedraOfFace.at(FaceKey(face[0], face[1], face[2])))
            {
                if (reached[neighbour]) continue;
                reached[neighbour] = true;
                pending.push_back(neighbour);
            }
    }

    std::vector<ElementId> connected;
    for (size_t index = 0; index < conflicting.size(); ++index)
        if (reached[index]) connected.push_back(conflicting[index]);
    return connected;
}

/// Whether point lies inside tetrahedron or on its boundary, exactly.
bool containsPoint(const TetrahedronElement& tetrahedron, const Point3D& point, const MeshData3D& meshData)
{
    auto& nodeIds = tetrahedron.getNodeIds();
    std::array<Point3D, 4> corners;
    for (size_t i = 0; i < 4; ++i)
        corners[i] = meshData.getNode(nodeIds[i])->getCoordinates();
    auto orientation = RobustPredicates3D::orientationSign(corners[0], corners[1], corners[2], corners[3]);
    if (orientation == 0) return false;

    for (size_t i = 0; i < 4; ++i)
    {
        auto replaced = corners;
        replaced[i] = point;
        auto sign = RobustPredicates3D::orientationSign(replaced[0], replaced[1], replaced[2], replaced[3]);
        if (sign == -orientation) return false;
    }
    return true;
}

} // namespace

} // namespace Meshing
