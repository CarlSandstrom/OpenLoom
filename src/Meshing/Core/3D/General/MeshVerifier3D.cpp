#include "Meshing/Core/3D/General/MeshVerifier3D.h"
#include "Common/TwinManager.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Core/3D/General/FacetTriangulation.h"
#include "Meshing/Core/3D/General/FacetTriangulationManager.h"
#include "Meshing/Data/2D/MeshData2D.h"
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
        if (!referencedNodes.contains(nodeId))
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

MeshVerificationResult MeshVerifier3D::verifyTwinConsistency(
    const TwinManager& twinManager,
    const FacetTriangulationManager& facetManager)
{
    MeshVerificationResult result;

    for (const auto& [key, value] : twinManager.getAllPairs())
    {
        const auto& [surfaceId, n1, n2] = key;
        const auto& [twinSurfaceId, m1, m2] = value;

        // Check both endpoint nodes exist on the source surface.
        const FacetTriangulation* facet = facetManager.getFacetTriangulation(surfaceId);
        if (!facet)
        {
            result.isValid = false;
            result.errors.push_back("Twin map references unknown surface: " + surfaceId);
            continue;
        }

        const MeshData2D& mesh = facet->getContext().getMeshData();
        if (!mesh.getNode(n1))
        {
            result.isValid = false;
            result.errors.push_back("Twin map node " + std::to_string(n1) +
                                    " on surface " + surfaceId + " not found in mesh");
        }
        if (!mesh.getNode(n2))
        {
            result.isValid = false;
            result.errors.push_back("Twin map node " + std::to_string(n2) +
                                    " on surface " + surfaceId + " not found in mesh");
        }

        // Check both endpoint nodes exist on the twin surface.
        const FacetTriangulation* twinFacet = facetManager.getFacetTriangulation(twinSurfaceId);
        if (!twinFacet)
        {
            result.isValid = false;
            result.errors.push_back("Twin map references unknown surface: " + twinSurfaceId);
            continue;
        }

        const MeshData2D& twinMesh = twinFacet->getContext().getMeshData();
        if (!twinMesh.getNode(m1))
        {
            result.isValid = false;
            result.errors.push_back("Twin map node " + std::to_string(m1) +
                                    " on surface " + twinSurfaceId + " not found in mesh");
        }
        if (!twinMesh.getNode(m2))
        {
            result.isValid = false;
            result.errors.push_back("Twin map node " + std::to_string(m2) +
                                    " on surface " + twinSurfaceId + " not found in mesh");
        }

        // Check symmetry: the reverse entry must exist and point back to (surfaceId, n1, n2).
        // Symmetry also implies equal subdivision counts: since recordSplit() always inserts
        // both new sub-segment pairs together, a symmetric map guarantees every sub-segment
        // on one side has exactly one twin on the other side.
        auto reverse = twinManager.getTwin(twinSurfaceId, m1, m2);
        if (!reverse)
        {
            result.isValid = false;
            result.errors.push_back("Twin map is not symmetric: no reverse entry for segment (" +
                                    std::to_string(m1) + ", " + std::to_string(m2) +
                                    ") on surface " + twinSurfaceId);
        }
        else
        {
            const auto& [reverseSurfaceId, r1, r2] = *reverse;
            if (reverseSurfaceId != surfaceId || r1 != n1 || r2 != n2)
            {
                result.isValid = false;
                result.errors.push_back("Twin map reverse entry mismatch for segment (" +
                                        std::to_string(m1) + ", " + std::to_string(m2) +
                                        ") on surface " + twinSurfaceId +
                                        ": expected back to (" + std::to_string(n1) + ", " +
                                        std::to_string(n2) + ") on " + surfaceId);
            }
        }
    }

    if (result.isValid)
    {
        SPDLOG_INFO("Twin consistency verification passed: {} directed pairs checked",
                    twinManager.getAllPairs().size());
    }
    else
    {
        SPDLOG_ERROR("Twin consistency verification failed with {} errors", result.errors.size());
    }

    return result;
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
