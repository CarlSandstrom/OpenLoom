#include "MeshVerifier3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "spdlog/spdlog.h"
#include <cmath>
#include <set>
#include <sstream>

namespace Meshing
{

MeshVerifier3D::MeshVerifier3D(const MeshData3D& meshData) :
    meshData_(meshData)
{
}

MeshVerificationResult MeshVerifier3D::verify() const
{
    lastResult_ = MeshVerificationResult{};
    lastResult_.nodeCount = meshData_.getNodeCount();
    lastResult_.elementCount = meshData_.getElementCount();

    // Check valid coordinates
    if (!verifyValidCoordinates())
    {
        lastResult_.isValid = false;
        lastResult_.errors.push_back("Found nodes with invalid (NaN/Inf) coordinates");
    }

    // Check node references
    if (!verifyNodeReferences())
    {
        lastResult_.isValid = false;
        lastResult_.errors.push_back("Found elements with invalid node references");
    }

    // Check for degenerate elements
    if (!verifyNoDegenerateElements())
    {
        lastResult_.isValid = false;
        std::ostringstream oss;
        oss << "Found " << lastResult_.degenerateElementCount << " degenerate element(s)";
        lastResult_.errors.push_back(oss.str());
    }

    // Check for inverted elements
    if (!verifyNoInvertedElements())
    {
        lastResult_.isValid = false;
        std::ostringstream oss;
        oss << "Found " << lastResult_.invertedElementCount << " inverted element(s)";
        lastResult_.errors.push_back(oss.str());
    }

    // Check for orphan nodes (warning only, not an error)
    if (!verifyNoOrphanNodes())
    {
        std::ostringstream oss;
        oss << "Found " << lastResult_.orphanNodeCount << " orphan node(s)";
        lastResult_.warnings.push_back(oss.str());
    }

    return lastResult_;
}

bool MeshVerifier3D::verifyNodeReferences() const
{
    for (const auto& [elemId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        const auto& nodeIds = tet->getNodeIds();
        for (size_t nodeId : nodeIds)
        {
            if (meshData_.getNode(nodeId) == nullptr)
            {
                spdlog::debug("MeshVerifier3D: Element {} references non-existent node {}", elemId, nodeId);
                return false;
            }
        }
    }
    return true;
}

bool MeshVerifier3D::verifyNoDegenerateElements() const
{
    size_t count = 0;
    for (const auto& [elemId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        const auto& nodeIds = tet->getNodeIds();
        double volume = computeSignedVolume(nodeIds[0], nodeIds[1], nodeIds[2], nodeIds[3]);

        if (std::abs(volume) < MIN_VALID_VOLUME)
        {
            spdlog::debug("MeshVerifier3D: Element {} is degenerate (volume = {:.2e})", elemId, volume);
            count++;
        }
    }

    lastResult_.degenerateElementCount = count;
    return count == 0;
}

bool MeshVerifier3D::verifyNoInvertedElements() const
{
    size_t count = 0;
    for (const auto& [elemId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        const auto& nodeIds = tet->getNodeIds();
        double volume = computeSignedVolume(nodeIds[0], nodeIds[1], nodeIds[2], nodeIds[3]);

        // If volume is significantly negative (not just numerical noise), it's inverted
        if (volume < -MIN_VALID_VOLUME)
        {
            spdlog::debug("MeshVerifier3D: Element {} is inverted (volume = {:.2e})", elemId, volume);
            count++;
        }
    }

    lastResult_.invertedElementCount = count;
    return count == 0;
}

bool MeshVerifier3D::verifyNoOrphanNodes() const
{
    // Collect all node IDs referenced by elements
    std::set<size_t> referencedNodes;
    for (const auto& [elemId, element] : meshData_.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        const auto& nodeIds = tet->getNodeIds();
        for (size_t nodeId : nodeIds)
        {
            referencedNodes.insert(nodeId);
        }
    }

    // Count nodes not referenced by any element
    size_t count = 0;
    for (const auto& [nodeId, node] : meshData_.getNodes())
    {
        if (referencedNodes.find(nodeId) == referencedNodes.end())
        {
            spdlog::debug("MeshVerifier3D: Node {} is orphaned", nodeId);
            count++;
        }
    }

    lastResult_.orphanNodeCount = count;
    return count == 0;
}

bool MeshVerifier3D::verifyValidCoordinates() const
{
    for (const auto& [nodeId, node] : meshData_.getNodes())
    {
        const Point3D& p = node->getCoordinates();

        if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z()))
        {
            spdlog::debug("MeshVerifier3D: Node {} has invalid coordinates ({}, {}, {})",
                          nodeId, p.x(), p.y(), p.z());
            return false;
        }
    }
    return true;
}

double MeshVerifier3D::computeSignedVolume(size_t nodeId1, size_t nodeId2,
                                           size_t nodeId3, size_t nodeId4) const
{
    const Node3D* n1 = meshData_.getNode(nodeId1);
    const Node3D* n2 = meshData_.getNode(nodeId2);
    const Node3D* n3 = meshData_.getNode(nodeId3);
    const Node3D* n4 = meshData_.getNode(nodeId4);

    if (!n1 || !n2 || !n3 || !n4)
    {
        return 0.0;
    }

    const Point3D& p1 = n1->getCoordinates();
    const Point3D& p2 = n2->getCoordinates();
    const Point3D& p3 = n3->getCoordinates();
    const Point3D& p4 = n4->getCoordinates();

    // Compute vectors from p1 to other vertices
    Point3D v1 = p2 - p1;
    Point3D v2 = p3 - p1;
    Point3D v3 = p4 - p1;

    // Signed volume = (1/6) * v1 . (v2 x v3)
    return v1.dot(v2.cross(v3)) / 6.0;
}

} // namespace Meshing
