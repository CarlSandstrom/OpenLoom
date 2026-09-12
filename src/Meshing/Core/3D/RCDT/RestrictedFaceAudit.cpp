#include "Meshing/Core/3D/RCDT/RestrictedFaceAudit.h"

#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/CurveSegmentManager.h"

#include <algorithm>
#include <array>
#include <numeric>
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
/// findNonManifoldEdges(), shared by that check and by removeExcessFaces().
struct RestrictedEdgeCoverage
{
    std::vector<FaceKey> incidentFaces;
    std::unordered_map<std::string, size_t> actualBySurface;

    /// How many faces the edge should carry in total. Always populated.
    size_t expectedCount = 0;

    /// Which surfaces those faces should belong to. Empty when the edge
    /// lies off any model curve AND its faces already disagree about the
    /// surface: the count is still expected to be 2, but the faces
    /// themselves pin down no surface to expect them on.
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

/// Every edge of the restricted-face set, with its incident faces and the
/// coverage the CAD topology expects of it.
RestrictedEdgeCoverageMap buildEdgeCoverage(const RestrictedFaceMap& restrictedFaces,
                                            const EdgeToAdjacentSurfacesMap& edgeToAdjacentSurfaces,
                                            const MeshData3D& meshData)
{
    RestrictedEdgeCoverageMap coverage;
    for (const auto& [face, surfaceId] : restrictedFaces)
    {
        for (const auto& edge : faceEdges(face))
        {
            auto& entry = coverage[edge];
            entry.incidentFaces.push_back(face);
            ++entry.actualBySurface[surfaceId];
        }
    }

    const auto& curveSegmentManager = meshData.getCurveSegmentManager();
    for (auto& [edge, entry] : coverage)
    {
        if (const auto fromTopology = expectedIncidentSurfaces(edge, curveSegmentManager, edgeToAdjacentSurfaces))
        {
            entry.expectedBySurface = *fromTopology;
            for (const auto& [surfaceId, expectedCount] : entry.expectedBySurface)
                entry.expectedCount += expectedCount;
        }
        else
        {
            // Off a model curve the topology fixes no expectation of its own,
            // so the surface-interior rule applies: 2 faces on one and the
            // same surface. WHICH surface is only pinned down when the faces
            // already agree -- when they don't, the count of 2 still stands
            // but there is no per-surface expectation left to check against.
            entry.expectedCount = 2;
            if (entry.actualBySurface.size() == 1)
                entry.expectedBySurface[entry.actualBySurface.begin()->first] = 2;
        }
    }
    return coverage;
}

/// Whether the edge joins two points on the same curve that are NOT
/// chain-adjacent -- a chord skipping over the curve's own intermediate
/// sample points.
bool isSameCurveChordEdge(size_t nodeIdA,
                          size_t nodeIdB,
                          const MeshData3D& meshData,
                          const EdgeToAdjacentSurfacesMap& edgeToAdjacentSurfaces)
{
    if (meshData.getCurveSegmentManager().findSegmentId(nodeIdA, nodeIdB))
        return false; // chain-adjacent -- a genuine protected edge, not a chord

    const auto& idsB = meshData.getGeometryIds(nodeIdB);
    for (const auto& geometryId : meshData.getGeometryIds(nodeIdA))
    {
        if (!edgeToAdjacentSurfaces.count(geometryId))
            continue; // not an edge (curve)-type geometryId -- a surface or corner tag
        if (std::find(idsB.begin(), idsB.end(), geometryId) != idsB.end())
            return true; // both endpoints on the same curve, but not chain-adjacent
    }
    return false;
}

bool hasSameCurveChordEdge(const FaceKey& face,
                           const MeshData3D& meshData,
                           const EdgeToAdjacentSurfacesMap& edgeToAdjacentSurfaces)
{
    const auto& n = face.nodeIds;
    return isSameCurveChordEdge(n[0], n[1], meshData, edgeToAdjacentSurfaces) ||
           isSameCurveChordEdge(n[0], n[2], meshData, edgeToAdjacentSurfaces) ||
           isSameCurveChordEdge(n[1], n[2], meshData, edgeToAdjacentSurfaces);
}

/// Removes every currently-restricted face that uses a same-curve chord
/// edge (see isSameCurveChordEdge()). Meant to be called ONCE, after
/// refinement has fully converged. Rejecting chord faces during
/// classification itself was tried first and measured to regress
/// SaddleSurfaceMesh badly (28->69 non-manifold edges): it denies
/// refinement's normal self-correcting process the chance to naturally
/// supersede most chord faces with the correct fine chain before this
/// runs (measured: buildInitial() alone had ~24 more chord faces than
/// the converged mesh's residual 15, most resolved on their own by
/// refinement's end). A post-hoc cleanup instead finds the correct
/// alternative already in place for that much smaller residual set --
/// see OPE-176 project memory for the full investigation.
///
/// A chord face is only dropped while its NON-chord edges can spare the
/// triangle -- i.e. none of them is at exactly 2. A face has 3 edges but
/// only the chord one is the problem; removing the face over that one
/// edge also takes the other two down with it, and an unconditional
/// version of this (measured on SaddleSurfaceMesh) tore fresh holes in
/// otherwise-healthy surface elsewhere, trading duplicates here for gaps
/// there. Applied greedily to a fixed point, since each removal changes
/// the counts the remaining candidates are judged against.
///
/// Returns the number of faces removed.
size_t removeChordFaces(RestrictedFaceMap& restrictedFaces,
                        BadRestrictedFaceMap& badFaces,
                        const EdgeToAdjacentSurfacesMap& edgeToAdjacentSurfaces,
                        const MeshData3D& meshData)
{
    std::vector<FaceKey> candidates;
    for (const auto& [face, surfaceId] : restrictedFaces)
        if (hasSameCurveChordEdge(face, meshData, edgeToAdjacentSurfaces))
            candidates.push_back(face);

    if (candidates.empty())
        return 0;

    std::unordered_map<EdgeKey, int, EdgeKeyHash> edgeCounts;
    for (const auto& [face, surfaceId] : restrictedFaces)
        for (const auto& edge : faceEdges(face))
            ++edgeCounts[edge];

    // Greedy, repeated to a fixed point: removing one candidate lowers its
    // edges' counts, which can make a previously-unsafe neighbouring
    // candidate safe (or vice versa), so a single pass in map order would
    // make the result depend on iteration order.
    size_t removed = 0;
    bool progress = true;
    while (progress)
    {
        progress = false;
        for (auto it = candidates.begin(); it != candidates.end();)
        {
            const FaceKey& face = *it;
            const auto& n = face.nodeIds;

            // Removing this face decrements all 3 of its edges. On the chord
            // edge that is the whole point. On a NON-chord edge it is only
            // acceptable while that edge can spare the triangle: dropping one
            // from exactly 2 to 1 tears open a fresh hole in an otherwise
            // healthy part of the surface, trading a duplicate here for a gap
            // there (measured on SaddleSurfaceMesh -- see project memory).
            bool safe = true;
            for (const auto& [nodeIdA, nodeIdB] : std::array<std::pair<size_t, size_t>, 3>{
                     std::make_pair(n[0], n[1]), std::make_pair(n[0], n[2]), std::make_pair(n[1], n[2])})
            {
                if (isSameCurveChordEdge(nodeIdA, nodeIdB, meshData, edgeToAdjacentSurfaces))
                    continue;
                if (edgeCounts[EdgeKey(nodeIdA, nodeIdB)] == 2)
                {
                    safe = false;
                    break;
                }
            }

            if (!safe)
            {
                ++it;
                continue;
            }

            for (const auto& edge : faceEdges(face))
                --edgeCounts[edge];
            restrictedFaces.erase(face);
            badFaces.erase(face);
            ++removed;
            it = candidates.erase(it);
            progress = true;
        }
    }
    return removed;
}

/// Post-hoc manifold enforcement: removes flaps of restricted faces that
/// cover a piece of surface already covered, leaving the edges they were
/// piled onto with exactly the number of faces the CAD topology calls for
/// (see findNonManifoldEdges() for that invariant). Meant to be called
/// ONCE, after refinement has fully converged and after removeChordFaces(),
/// whose removals change the counts this judges.
///
/// A flap is identified as a whole CONNECTED COMPONENT rather than
/// face-by-face. Faces are joined through every edge that is NOT
/// over-covered, so an over-covered edge acts as a cut: a flap laid over
/// an otherwise correct sheet meets that sheet only along over-covered
/// edges -- its own interior edges carry just its own two triangles --
/// and so falls out as a component of its own. This is what OPE-184
/// measured the defect to be: not 222 independent faults but 21 doubled
/// patches of 4-7 elements across, the seam between patch and sheet
/// showing up as the multiplicity-3 edges.
///
/// A component is removed only when it touches an over-covered edge at
/// all, and only while every edge it touches can spare its faces: what
/// remains after the removal must still meet the expected count, or must
/// be nothing at all (a flap's own interior edges leave the restricted
/// set along with it, so they cannot be left as holes). The largest
/// component is never removed. Applied greedily to a fixed point, since
/// each removal changes the counts the remaining candidates are judged
/// against.
///
/// Deliberately NOT "for any edge with more than 2 faces, keep the best
/// 2 and drop the rest": the expected count is read off the topology,
/// so an edge where three surfaces genuinely meet -- a triple line in a
/// multi-material model -- expects 3 and is never touched here.
///
/// Returns the number of faces removed.
size_t removeExcessFaces(RestrictedFaceMap& restrictedFaces,
                         BadRestrictedFaceMap& badFaces,
                         const EdgeToAdjacentSurfacesMap& edgeToAdjacentSurfaces,
                         const MeshData3D& meshData)
{
    size_t removed = 0;
    bool progress = true;
    while (progress)
    {
        progress = false;

        const auto coverage = buildEdgeCoverage(restrictedFaces, edgeToAdjacentSurfaces, meshData);

        std::unordered_map<FaceKey, size_t, FaceKeyHash> faceIndices;
        std::vector<FaceKey> faces;
        faces.reserve(restrictedFaces.size());
        for (const auto& [face, surfaceId] : restrictedFaces)
        {
            faceIndices.emplace(face, faces.size());
            faces.push_back(face);
        }
        if (faces.empty())
            break;

        // Union-find over the faces, joining them across every edge that is
        // NOT over-covered. An over-covered edge is left as a cut, which is
        // what separates a flap from the sheet it was laid over.
        std::vector<size_t> parents(faces.size());
        std::iota(parents.begin(), parents.end(), size_t{0});
        auto findRoot = [&parents](size_t index)
        {
            while (parents[index] != index)
            {
                parents[index] = parents[parents[index]];
                index = parents[index];
            }
            return index;
        };

        for (const auto& [edge, entry] : coverage)
        {
            if (entry.incidentFaces.size() > entry.expectedCount)
                continue;
            for (size_t i = 1; i < entry.incidentFaces.size(); ++i)
            {
                const size_t rootA = findRoot(faceIndices.at(entry.incidentFaces[0]));
                const size_t rootB = findRoot(faceIndices.at(entry.incidentFaces[i]));
                if (rootA != rootB)
                    parents[rootB] = rootA;
            }
        }

        std::unordered_map<size_t, std::vector<FaceKey>> components;
        for (size_t index = 0; index < faces.size(); ++index)
            components[findRoot(index)].push_back(faces[index]);

        // Components are ranked by size, ties broken on their smallest face.
        // Size alone leaves the outcome to restrictedFaces's hash order in
        // any configuration where several components are equally removable,
        // which would make the mesh depend on iteration order -- the same
        // hazard removeChordFaces() repeats itself to a fixed point to avoid.
        for (auto& [root, componentFaces] : components)
            std::sort(componentFaces.begin(), componentFaces.end());

        const auto ranking = [&components](size_t root)
        { return std::make_pair(components.at(root).size(), components.at(root).front()); };

        size_t largestComponentRoot = components.begin()->first;
        for (const auto& [root, componentFaces] : components)
        {
            if (ranking(largestComponentRoot) < ranking(root))
                largestComponentRoot = root;
        }

        // Smallest first: a flap is small, and testing it before anything
        // larger keeps a big component from being judged against counts a
        // smaller one has already been credited with.
        std::vector<size_t> roots;
        for (const auto& [root, componentFaces] : components)
        {
            if (root != largestComponentRoot)
                roots.push_back(root);
        }
        std::sort(roots.begin(),
                  roots.end(),
                  [&ranking](size_t rootA, size_t rootB)
                  { return ranking(rootA) < ranking(rootB); });

        for (const size_t root : roots)
        {
            const auto& componentFaces = components.at(root);

            std::unordered_map<EdgeKey, size_t, EdgeKeyHash> contributed;
            for (const auto& face : componentFaces)
                for (const auto& edge : faceEdges(face))
                    ++contributed[edge];

            // Only a component that is actually piled on top of something is
            // a candidate: one touching no over-covered edge is ordinary
            // surface, however small, and removing it would tear a hole.
            bool touchesOverCoveredEdge = false;
            bool safe = true;
            for (const auto& [edge, contributedCount] : contributed)
            {
                const auto& entry = coverage.at(edge);
                if (entry.incidentFaces.size() > entry.expectedCount)
                    touchesOverCoveredEdge = true;

                // What is left on the edge must still meet the expected count
                // -- or be nothing at all, which is the flap's own interior
                // edges leaving the restricted set along with it rather than
                // being left behind as holes.
                const size_t remaining = entry.incidentFaces.size() - contributedCount;
                if (remaining != 0 && remaining < entry.expectedCount)
                {
                    safe = false;
                    break;
                }
            }

            if (!touchesOverCoveredEdge || !safe)
                continue;

            for (const auto& face : componentFaces)
            {
                restrictedFaces.erase(face);
                badFaces.erase(face);
                ++removed;
            }
            progress = true;
            break;
        }
    }
    return removed;
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

DefectiveFaceRemovalSummary removeDefectiveFaces(RestrictedFaceMap& restrictedFaces,
                                                 BadRestrictedFaceMap& badFaces,
                                                 const EdgeToAdjacentSurfacesMap& edgeToAdjacentSurfaces,
                                                 const MeshData3D& meshData)
{
    DefectiveFaceRemovalSummary summary;
    summary.chordFacesRemoved = removeChordFaces(restrictedFaces, badFaces, edgeToAdjacentSurfaces, meshData);
    summary.excessFacesRemoved = removeExcessFaces(restrictedFaces, badFaces, edgeToAdjacentSurfaces, meshData);

    for (const auto& nonManifoldEdge : findNonManifoldEdges(restrictedFaces, edgeToAdjacentSurfaces, meshData))
    {
        switch (nonManifoldEdge.defect)
        {
        case RestrictedEdgeDefect::MissingFace:
            ++summary.remainingMissingFaceEdges;
            break;
        case RestrictedEdgeDefect::ExcessFace:
            ++summary.remainingExcessFaceEdges;
            break;
        case RestrictedEdgeDefect::SurfaceMismatch:
            ++summary.remainingSurfaceMismatchEdges;
            break;
        }
    }
    return summary;
}

} // namespace RestrictedFaceAudit

} // namespace Meshing
