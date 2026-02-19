#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Common/DebugFlags.h"
#include "Common/Exceptions/MeshException.h"
#include "Meshing/Core/3D/General/ElementQuality3D.h"
#include "Export/VtkExporter.h"
#include "Meshing/Core/3D/General/MeshQueries3D.h"
#include "Meshing/Core/3D/General/MeshVerifier3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "spdlog/spdlog.h"
#include <sstream>

namespace Meshing
{

namespace
{

const char* phaseToString(MeshingPhase3D phase)
{
    switch (phase)
    {
    case MeshingPhase3D::InitialDelaunay:
        return "InitialDelaunay";
    case MeshingPhase3D::SegmentRecovery:
        return "SegmentRecovery";
    case MeshingPhase3D::FacetRecovery:
        return "FacetRecovery";
    case MeshingPhase3D::ExteriorRemoved:
        return "ExteriorRemoved";
    case MeshingPhase3D::Refined:
        return "Refined";
    case MeshingPhase3D::PostProcessed:
        return "PostProcessed";
    default:
        return "Unknown";
    }
}

void verifyBasicIntegrity(const MeshData3D& meshData, std::vector<std::string>& errors)
{
    MeshVerifier3D verifier(meshData);
    auto result = verifier.verify();

    if (!result.isValid)
    {
        for (const auto& error : result.errors)
        {
            errors.push_back(error);
        }
    }

    for (const auto& warning : result.warnings)
    {
        spdlog::warn("MeshDebugUtils3D: {}", warning);
    }
}

void verifySubsegmentsPresent(const MeshData3D& meshData, std::vector<std::string>& errors)
{
    MeshQueries3D queries(meshData);
    const auto& subsegments = meshData.getConstrainedSubsegments();
    size_t missingCount = 0;

    for (const auto& subsegment : subsegments)
    {
        if (!queries.edgeExistsInMesh(subsegment.nodeId1, subsegment.nodeId2))
        {
            missingCount++;
            if (missingCount <= 5)
            {
                spdlog::debug("MeshDebugUtils3D: Subsegment ({}, {}) missing from mesh",
                              subsegment.nodeId1, subsegment.nodeId2);
            }
        }
    }

    if (missingCount > 0)
    {
        std::ostringstream oss;
        oss << missingCount << " of " << subsegments.size()
            << " subsegment(s) missing from mesh edges";
        errors.push_back(oss.str());
    }
}

void verifySubfacetsPresent(const MeshData3D& meshData, std::vector<std::string>& errors)
{
    MeshQueries3D queries(meshData);
    const auto& subfacets = meshData.getConstrainedSubfacets();
    size_t missingCount = 0;

    for (const auto& subfacet : subfacets)
    {
        if (!queries.faceExistsInMesh(subfacet.nodeId1, subfacet.nodeId2, subfacet.nodeId3))
        {
            missingCount++;
            if (missingCount <= 5)
            {
                spdlog::debug("MeshDebugUtils3D: Subfacet ({}, {}, {}) missing from mesh",
                              subfacet.nodeId1, subfacet.nodeId2, subfacet.nodeId3);
            }
        }
    }

    if (missingCount > 0)
    {
        std::ostringstream oss;
        oss << missingCount << " of " << subfacets.size()
            << " subfacet(s) missing from mesh faces";
        errors.push_back(oss.str());
    }
}

void verifyQualityBound(const MeshData3D& meshData, double qualityBound,
                         std::vector<std::string>& errors)
{
    if (qualityBound <= 0.0)
    {
        return;
    }

    ElementQuality3D quality(meshData);
    auto skinnyTets = quality.getSkinnyTetrahedraSortedByQuality(qualityBound);

    if (!skinnyTets.empty())
    {
        std::ostringstream oss;
        oss << skinnyTets.size() << " tetrahedra exceed quality bound B=" << qualityBound;
        if (!skinnyTets.empty())
        {
            oss << " (worst ratio: " << skinnyTets.front().second << ")";
        }
        errors.push_back(oss.str());
    }
}

} // namespace

void exportAndVerifyMesh3D(MeshData3D& meshData,
                           MeshingPhase3D phase,
                           const std::string& filenamePrefix,
                           size_t& exportCounter,
                           double qualityBound)
{
    if (CMESH_DEBUG_ENABLED(EXPORT_MESH_EACH_ITERATION))
    {
        Export::VtkExporter exporter;
        exporter.exportMesh(meshData, filenamePrefix + "_" + std::to_string(exportCounter++) + ".vtu");
    }

    if (CMESH_DEBUG_ENABLED(CHECK_MESH_EACH_ITERATION))
    {
        spdlog::info("{} [{}]: Verifying mesh at step {} ({} nodes, {} elements)",
                     filenamePrefix, phaseToString(phase), exportCounter,
                     meshData.getNodeCount(), meshData.getElementCount());

        std::vector<std::string> errors;

        // All phases: basic mesh integrity
        verifyBasicIntegrity(meshData, errors);

        // SegmentRecovery and later: all subsegments must be mesh edges
        if (phase >= MeshingPhase3D::SegmentRecovery)
        {
            verifySubsegmentsPresent(meshData, errors);
        }

        // FacetRecovery and later: all subfacets must be mesh faces
        if (phase >= MeshingPhase3D::FacetRecovery)
        {
            verifySubfacetsPresent(meshData, errors);
        }

        // Refined and later: quality bound satisfied
        if (phase >= MeshingPhase3D::Refined)
        {
            verifyQualityBound(meshData, qualityBound, errors);
        }

        if (!errors.empty())
        {
            for (const auto& error : errors)
            {
                spdlog::error(" - {}", error);
            }
            CMESH_THROW_VERIFICATION_FAILED("3D mesh verification failed at phase "
                                                 + std::string(phaseToString(phase)),
                                             errors);
        }
    }
}

} // namespace Meshing
