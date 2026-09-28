#include "Meshing/Core/3D/RCDT/RestrictedFaceAudit.h"

#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/CurveSegmentManager.h"

#include <array>
#include <optional>
#include <utility>

namespace Meshing
{

namespace
{

std::array<EdgeKey, 3> faceEdges(const FaceKey& face)
{
    const auto& n = face.nodeIds;
    return {EdgeKey(n[0], n[1]), EdgeKey(n[0], n[2]), EdgeKey(n[1], n[2])};
}

/// One edge's restricted-face coverage set against what the CAD topology
/// calls for there -- the raw material of the invariant documented on
/// findNonManifoldEdges().
struct RestrictedEdgeCoverage
{
    std::unordered_map<std::string, size_t> actualBySurface;

    /// Which surfaces the edge's faces should belong to, and how many on
    /// each. Empty when the edge lies off any model curve AND its faces
    /// already disagree about the surface: 2 faces are still expected, but
    /// the faces themselves pin down no surface to expect them on.
    std::unordered_map<std::string, size_t> expectedBySurface;
};

using RestrictedEdgeCoverageMap = std::unordered_map<EdgeKey, RestrictedEdgeCoverage, EdgeKeyHash>;

std::optional<std::unordered_map<std::string, size_t>> expectedIncidentSurfaces(
    const EdgeKey& edge,
    const CurveSegmentManager& curveSegmentManager,
    const EdgeToAdjacentSurfacesMap& edgeToAdjacentSurfaces)
{
    const auto segmentId = curveSegmentManager.findSegmentId(edge.nodeIds[0], edge.nodeIds[1]);
    if (!segmentId)
        return std::nullopt;

    const auto adjacentIt = edgeToAdjacentSurfaces.find(curveSegmentManager.getSegment(*segmentId).edgeId);
    if (adjacentIt == edgeToAdjacentSurfaces.end() || adjacentIt->second.empty())
        return std::nullopt;

    // A seam curve is listed twice against its own surface -- it bounds that
    // surface's UV domain on both sides -- which is exactly the expectation
    // of 2 same-surface faces it should carry, so the multiset is taken as
    // it comes rather than deduplicated.
    std::unordered_map<std::string, size_t> expected;
    for (const auto& surfaceId : adjacentIt->second)
        ++expected[surfaceId];
    return expected;
}

/// Every edge of the restricted-face set, with its coverage and the coverage
/// the CAD topology expects of it.
RestrictedEdgeCoverageMap buildEdgeCoverage(const RestrictedFaceMap& restrictedFaces,
                                            const EdgeToAdjacentSurfacesMap& edgeToAdjacentSurfaces,
                                            const MeshData3D& meshData)
{
    RestrictedEdgeCoverageMap coverage;
    for (const auto& [face, surfaceId] : restrictedFaces)
    {
        for (const auto& edge : faceEdges(face))
        {
            ++coverage[edge].actualBySurface[surfaceId];
        }
    }

    const auto& curveSegmentManager = meshData.getCurveSegmentManager();
    for (auto& [edge, entry] : coverage)
    {
        if (const auto fromTopology = expectedIncidentSurfaces(edge, curveSegmentManager, edgeToAdjacentSurfaces))
            entry.expectedBySurface = *fromTopology;
        else
        {
            // Off a model curve the topology fixes no expectation of its own,
            // so the surface-interior rule applies: 2 faces on one and the
            // same surface. WHICH surface is only pinned down when the faces
            // already agree -- when they don't, the count of 2 still stands
            // but there is no per-surface expectation left to check against.
            if (entry.actualBySurface.size() == 1)
                entry.expectedBySurface[entry.actualBySurface.begin()->first] = 2;
        }
    }
    return coverage;
}

} // namespace

namespace RestrictedFaceAudit
{

std::vector<NonManifoldRestrictedEdge> findNonManifoldEdges(
    const RestrictedFaceMap& restrictedFaces,
    const EdgeToAdjacentSurfacesMap& edgeToAdjacentSurfaces,
    const MeshData3D& meshData)
{
    std::vector<NonManifoldRestrictedEdge> nonManifoldEdges;
    for (const auto& [edge, entry] : buildEdgeCoverage(restrictedFaces, edgeToAdjacentSurfaces, meshData))
    {
        if (entry.expectedBySurface.empty())
        {
            nonManifoldEdges.push_back(
                {edge, entry.actualBySurface.begin()->first, RestrictedEdgeDefect::SurfaceMismatch});
            continue;
        }

        // A surface short of a face is reported in preference to one carrying
        // an excess: it names where a repair point has to be projected, while
        // an excess names only where one already is.
        std::optional<std::string> missingSurfaceId;
        std::optional<std::string> excessSurfaceId;
        for (const auto& [surfaceId, expectedCount] : entry.expectedBySurface)
        {
            const auto actualIt = entry.actualBySurface.find(surfaceId);
            const size_t actualCount = actualIt == entry.actualBySurface.end() ? 0 : actualIt->second;
            if (actualCount < expectedCount && !missingSurfaceId)
                missingSurfaceId = surfaceId;
            else if (actualCount > expectedCount && !excessSurfaceId)
                excessSurfaceId = surfaceId;
        }
        for (const auto& actualEntry : entry.actualBySurface)
        {
            // A surface not expected here at all: every face it contributes
            // is an excess one.
            if (!entry.expectedBySurface.count(actualEntry.first) && !excessSurfaceId)
                excessSurfaceId = actualEntry.first;
        }

        if (missingSurfaceId)
        {
            // Short on one surface and long on another is the expected number
            // of faces landing on the wrong surfaces, not a hole beside a
            // duplicate -- repair still projects onto the surface that is short.
            const auto defect =
                excessSurfaceId ? RestrictedEdgeDefect::SurfaceMismatch : RestrictedEdgeDefect::MissingFace;
            nonManifoldEdges.push_back({edge, *missingSurfaceId, defect});
        }
        else if (excessSurfaceId)
        {
            nonManifoldEdges.push_back({edge, *excessSurfaceId, RestrictedEdgeDefect::ExcessFace});
        }
    }
    return nonManifoldEdges;
}

} // namespace RestrictedFaceAudit

} // namespace Meshing
