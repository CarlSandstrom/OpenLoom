#include "MeshQueries3D.h"
#include "ConstraintChecker3D.h"
#include "ElementGeometry3D.h"
#include "ElementQuality3D.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <map>
#include <unordered_set>

namespace Meshing
{

MeshQueries3D::MeshQueries3D(const MeshData3D& meshData) :
    meshData_(meshData)
{
}

std::vector<size_t> MeshQueries3D::findConflictingTetrahedra(const Point3D& point) const
{
    std::vector<size_t> conflicting;
    ElementGeometry3D geometry(meshData_);

    // Check all tetrahedra
    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
        {
            continue;
        }

        // Check if point is inside the circumsphere
        if (geometry.isPointInsideCircumscribingSphere(*tet, point))
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

bool MeshQueries3D::edgeExistsInMesh(size_t nodeId1, size_t nodeId2) const
{
    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        const auto& nodes = tet->getNodeIds();
        bool hasNode1 = std::find(nodes.begin(), nodes.end(), nodeId1) != nodes.end();
        bool hasNode2 = std::find(nodes.begin(), nodes.end(), nodeId2) != nodes.end();

        if (hasNode1 && hasNode2)
        {
            return true;
        }
    }
    return false;
}

std::vector<size_t> MeshQueries3D::findTetrahedraWithEdge(size_t nodeId1, size_t nodeId2) const
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

        if (hasNode1 && hasNode2)
        {
            result.push_back(tetId);
        }
    }
    return result;
}

bool MeshQueries3D::faceExistsInMesh(size_t nodeId1, size_t nodeId2, size_t nodeId3) const
{
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
            return true;
        }
    }
    return false;
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

std::vector<size_t> MeshQueries3D::findSkinnyTetrahedra(double ratioBound) const
{
    std::vector<size_t> skinnyTets;
    ElementQuality3D quality(meshData_);

    for (const auto& [tetId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        double ratio = quality.getCircumradiusToShortestEdgeRatio(*tet);
        if (ratio > ratioBound)
        {
            skinnyTets.push_back(tetId);
        }
    }

    return skinnyTets;
}

std::vector<ConstrainedSubsegment3D> MeshQueries3D::findEncroachingSubsegments(
    const Point3D& point,
    const std::vector<ConstrainedSubsegment3D>& subsegments) const
{
    std::vector<ConstrainedSubsegment3D> encroached;
    ConstraintChecker3D checker(meshData_);

    for (const auto& subsegment : subsegments)
    {
        if (checker.isSubsegmentEncroached(subsegment, point))
        {
            encroached.push_back(subsegment);
        }
    }

    return encroached;
}

std::vector<ConstrainedSubfacet3D> MeshQueries3D::findEncroachingSubfacets(
    const Point3D& point,
    const std::vector<ConstrainedSubfacet3D>& subfacets) const
{
    std::vector<ConstrainedSubfacet3D> encroached;
    ConstraintChecker3D checker(meshData_);

    for (const auto& subfacet : subfacets)
    {
        if (checker.isSubfacetEncroached(subfacet, point))
        {
            encroached.push_back(subfacet);
        }
    }

    return encroached;
}

std::vector<ConstrainedSubsegment3D> MeshQueries3D::extractConstrainedSubsegments(
    const Topology3D::Topology3D& topology,
    const std::map<std::string, size_t>& cornerIdToPointIndexMap,
    const std::map<size_t, size_t>& pointIndexToNodeIdMap,
    const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap) const
{
    std::vector<ConstrainedSubsegment3D> constrainedSubsegments;

    for (const auto& edgeId : topology.getAllEdgeIds())
    {
        auto edgePointsIt = edgeIdToPointIndicesMap.find(edgeId);

        if (edgePointsIt != edgeIdToPointIndicesMap.end() && edgePointsIt->second.size() >= 2)
        {
            const auto& pointIndices = edgePointsIt->second;

            for (size_t i = 0; i < pointIndices.size() - 1; ++i)
            {
                size_t startPointIdx = pointIndices[i];
                size_t endPointIdx = pointIndices[i + 1];

                size_t startNodeId = pointIndexToNodeIdMap.at(startPointIdx);
                size_t endNodeId = pointIndexToNodeIdMap.at(endPointIdx);

                constrainedSubsegments.push_back(
                    ConstrainedSubsegment3D{startNodeId, endNodeId, edgeId});

                spdlog::debug("Edge {} subsegment {}: Node IDs ({}, {})",
                              edgeId, i, startNodeId, endNodeId);
            }
        }
        else
        {
            // Edge has no intermediate points; create single subsegment from corners
            const auto& edgeTopology = topology.getEdge(edgeId);

            auto startCornerIt = cornerIdToPointIndexMap.find(edgeTopology.getStartCornerId());
            auto endCornerIt = cornerIdToPointIndexMap.find(edgeTopology.getEndCornerId());

            if (startCornerIt != cornerIdToPointIndexMap.end() &&
                endCornerIt != cornerIdToPointIndexMap.end())
            {
                size_t startNodeId = pointIndexToNodeIdMap.at(startCornerIt->second);
                size_t endNodeId = pointIndexToNodeIdMap.at(endCornerIt->second);

                constrainedSubsegments.push_back(
                    ConstrainedSubsegment3D{startNodeId, endNodeId, edgeId});

                spdlog::debug("Edge {}: Node IDs ({}, {})", edgeId, startNodeId, endNodeId);
            }
        }
    }

    return constrainedSubsegments;
}

} // namespace Meshing
