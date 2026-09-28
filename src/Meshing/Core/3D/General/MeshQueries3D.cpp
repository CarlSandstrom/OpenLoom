#include "Meshing/Core/3D/General/MeshQueries3D.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/General/RegularPredicates3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <map>

namespace Meshing
{

MeshQueries3D::MeshQueries3D(const MeshData3D& meshData) :
    meshData_(meshData)
{
}

std::vector<size_t> MeshQueries3D::findConflictingTetrahedra(const Point3D& point, double pointWeight) const
{
    std::vector<size_t> conflicting;

    // Check all tetrahedra
    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
        {
            continue;
        }

        // Weighted in-sphere ("orthosphere") test via a robust exact-
        // arithmetic determinant sign, not an explicit circumcenter/radius:
        // for a near-degenerate (nearly flat) tetrahedron, solving for the
        // circumcenter is a near-singular linear system whose solution can
        // be wrong by orders of magnitude, which silently corrupts this
        // conflict search (see RobustPredicates3D, OPE-159). Reduces
        // exactly to the plain (unweighted) test when every weight involved
        // is 0 (see RegularPredicates3D, OPE-176).
        const auto& nodeIds = tet->getNodeIds();
        const Node3D* n0 = meshData_.getNode(nodeIds[0]);
        const Node3D* n1 = meshData_.getNode(nodeIds[1]);
        const Node3D* n2 = meshData_.getNode(nodeIds[2]);
        const Node3D* n3 = meshData_.getNode(nodeIds[3]);
        if (!n0 || !n1 || !n2 || !n3)
        {
            continue;
        }

        if (RegularPredicates3D::insidePointOrthosphere(
                n0->getCoordinates(), n0->getWeight(),
                n1->getCoordinates(), n1->getWeight(),
                n2->getCoordinates(), n2->getWeight(),
                n3->getCoordinates(), n3->getWeight(),
                point, pointWeight))
        {
            conflicting.push_back(tetId);
        }
    }

    return conflicting;
}

std::vector<std::array<size_t, 3>>
MeshQueries3D::findCavityBoundary(const std::vector<size_t>& conflictingIndices) const
{
    // Map to count how many times each face appears, and store the original oriented face
    std::map<FaceKey, size_t> faceCount;
    std::map<FaceKey, std::array<size_t, 3>> faceOrientation;

    // Iterate through all conflicting tetrahedra and count their faces
    for (size_t tetId : conflictingIndices)
    {
        const auto* element = meshData_.getElement(tetId);
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element);
        if (!tet)
        {
            continue;
        }

        // Get the four faces with consistent outward-pointing orientation
        // Using standard tet face ordering (each face opposite to one node)
        auto faces = tet->getFaces();

        for (const auto& face : faces)
        {
            FaceKey key(face[0], face[1], face[2]);
            faceCount[key]++;
            // Store the oriented face (first occurrence wins; for boundary faces
            // there is only one occurrence)
            if (!faceOrientation.contains(key))
            {
                faceOrientation[key] = face;
            }
        }
    }

    // Boundary faces appear exactly once (not shared with another conflicting tet)
    std::vector<std::array<size_t, 3>> boundary;
    for (const auto& [faceKey, count] : faceCount)
    {
        if (count == 1)
        {
            boundary.push_back(faceOrientation[faceKey]);
        }
    }

    return boundary;
}

std::vector<size_t> MeshQueries3D::findTetrahedraWithFace(size_t nodeId1, size_t nodeId2, size_t nodeId3) const
{
    std::vector<size_t> result;

    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        const auto& nodes = tet->getNodeIds();
        bool hasNode1 = std::find(nodes.begin(), nodes.end(), nodeId1) != nodes.end();
        bool hasNode2 = std::find(nodes.begin(), nodes.end(), nodeId2) != nodes.end();
        bool hasNode3 = std::find(nodes.begin(), nodes.end(), nodeId3) != nodes.end();

        if (hasNode1 && hasNode2 && hasNode3)
        {
            result.push_back(tetId);
        }
    }
    return result;
}

size_t MeshQueries3D::findOppositeVertex(size_t tetId, size_t faceNode1, size_t faceNode2, size_t faceNode3) const
{
    const auto* element = meshData_.getElement(tetId);
    const auto* tet = dynamic_cast<const TetrahedralElement*>(element);
    if (!tet)
    {
        return SIZE_MAX;
    }

    const auto& nodes = tet->getNodeIds();
    for (size_t nodeId : nodes)
    {
        if (nodeId != faceNode1 && nodeId != faceNode2 && nodeId != faceNode3)
        {
            return nodeId;
        }
    }
    return SIZE_MAX;
}

} // namespace Meshing
