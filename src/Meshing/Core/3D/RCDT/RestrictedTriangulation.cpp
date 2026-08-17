#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Geometry/3D/Base/IVolume3D.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/ElementQuality3D.h"
#include "Meshing/Core/3D/RCDT/RCDTQualityController.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "Topology/Topology3D.h"

#include <algorithm>

namespace Meshing
{

namespace
{

constexpr size_t INVALID_ID = SIZE_MAX;

// The tessellation oracle's target cell size is this fraction of minimumEdgeLength.
// Cells smaller than minimumEdgeLength / 2 are guaranteed fine enough to
// correctly classify any face whose shortest edge is at or above that floor.
constexpr double TESSELLATION_CELL_SIZE_FACTOR = 0.5;

std::array<EdgeKey, 3> faceEdges(const FaceKey& face)
{
    const auto& n = face.nodeIds;
    return {EdgeKey(n[0], n[1]), EdgeKey(n[0], n[2]), EdgeKey(n[1], n[2])};
}

enum class PointPhaseKind
{
    Exterior,
    InVolume,
    Ambiguous
};

struct PointPhase
{
    PointPhaseKind kind = PointPhaseKind::Exterior;
    std::string volumeId;
};

PointPhase classifyPointPhase(const Point3D& point,
                              const std::vector<std::string>& volumeIds,
                              const Geometry3D::GeometryCollection3D& geometry)
{
    std::optional<std::string> insideVolumeId;
    for (const auto& volumeId : volumeIds)
    {
        const Geometry3D::IVolume3D* volume = geometry.getVolume(volumeId);
        if (!volume)
            continue;

        switch (volume->classifyPoint(point))
        {
        case Geometry3D::VolumeClassification::Inside:
            if (insideVolumeId)
                return {PointPhaseKind::Ambiguous, {}};
            insideVolumeId = volumeId;
            break;
        case Geometry3D::VolumeClassification::OnBoundary:
        case Geometry3D::VolumeClassification::Unknown:
            return {PointPhaseKind::Ambiguous, {}};
        case Geometry3D::VolumeClassification::Outside:
            break;
        }
    }

    if (insideVolumeId)
        return {PointPhaseKind::InVolume, *insideVolumeId};
    return {PointPhaseKind::Exterior, {}};
}

} // namespace

void RestrictedTriangulation::buildFrom(const MeshData3D& meshData,
                                        const MeshConnectivity& connectivity,
                                        const Geometry3D::GeometryCollection3D& geometry,
                                        const Topology3D::Topology3D& topology,
                                        double minimumEdgeLength,
                                        const SurfaceMesh3DQualitySettings& settings)
{
    settings_ = settings;
    restrictedFaces_.clear();
    badFaces_.clear();
    surfaceIds_.clear();
    edgeToAdjacentSurfaces_.clear();
    surfaceTessellations_.clear();
    volumeIds_.clear();
    periodicSurfaceIds_.clear();

    for (const auto& surfaceId : topology.getAllSurfaceIds())
        surfaceIds_.insert(surfaceId);

    volumeIds_ = topology.getAllVolumeIds();

    for (const auto& surfaceId : surfaceIds_)
    {
        for (const auto& edgeId : topology.getSurface(surfaceId).getBoundaryEdgeIds())
        {
            if (topology.getSeamCollection().isSeamTwin(edgeId))
            {
                periodicSurfaceIds_.insert(surfaceId);
                break;
            }
        }
    }

    for (const auto& edgeId : topology.getAllEdgeIds())
        edgeToAdjacentSurfaces_[edgeId] = topology.getEdge(edgeId).getAdjacentSurfaceIds();

    for (const auto& cornerId : topology.getAllCornerIds())
    {
        const auto& connectedSurfaces = topology.getCorner(cornerId).getConnectedSurfaceIds();
        cornerToAdjacentSurfaces_[cornerId] =
            std::vector<std::string>(connectedSurfaces.begin(), connectedSurfaces.end());
    }

    const double targetCellSize = minimumEdgeLength * TESSELLATION_CELL_SIZE_FACTOR;
    for (const auto& surfaceId : surfaceIds_)
    {
        const Geometry3D::ISurface3D* surface = geometry.getSurface(surfaceId);
        if (!surface)
            continue;
        surfaceTessellations_[surfaceId].build(*surface, targetCellSize);
    }

    for (const auto& [elementId, element] : meshData.getElements())
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tet)
            continue;

        for (const auto& faceArray : tet->getFaces())
        {
            const FaceKey face(faceArray);
            if (restrictedFaces_.count(face))
                continue;

            auto surfaceId = classifyFace(face, meshData, connectivity, geometry);
            if (surfaceId)
            {
                updateBadFaceEntry(face, *surfaceId, meshData, geometry);
                restrictedFaces_.emplace(face, std::move(*surfaceId));
            }
        }
    }
}

void RestrictedTriangulation::updateAfterInsertion(
    const std::vector<FaceKey>& cavityInteriorFaceKeys,
    size_t newNodeId,
    const MeshData3D& meshData,
    const MeshConnectivity& connectivity,
    const Geometry3D::GeometryCollection3D& geometry)
{
    for (const auto& face : cavityInteriorFaceKeys)
    {
        restrictedFaces_.erase(face);
        badFaces_.erase(face);
    }

    for (const size_t elementId : connectivity.getNodeElements(newNodeId))
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId));
        if (!tet)
            continue;

        for (const auto& faceArray : tet->getFaces())
        {
            // Always reclassify, even if already present: 3 of this tet's 4
            // faces contain newNodeId and are genuinely new, but the 4th
            // (opposite newNodeId) is an existing cavity-boundary face whose
            // OTHER neighboring tetrahedron didn't change — only THIS one
            // did. Skipping it here because it already has a stored
            // classification is what let that classification go stale: its
            // dual Voronoi edge is defined by both neighbors, and one of
            // them just changed.
            const FaceKey face(faceArray);
            auto surfaceId = classifyFace(face, meshData, connectivity, geometry);
            if (surfaceId)
            {
                updateBadFaceEntry(face, *surfaceId, meshData, geometry);
                restrictedFaces_.insert_or_assign(face, std::move(*surfaceId));
            }
            else
            {
                restrictedFaces_.erase(face);
                badFaces_.erase(face);
            }
        }
    }
}

void RestrictedTriangulation::invalidateFacesWithEdge(size_t nodeId1, size_t nodeId2)
{
    for (auto it = restrictedFaces_.begin(); it != restrictedFaces_.end();)
    {
        const auto& ids = it->first.nodeIds;
        const bool hasNode1 = ids[0] == nodeId1 || ids[1] == nodeId1 || ids[2] == nodeId1;
        const bool hasNode2 = ids[0] == nodeId2 || ids[1] == nodeId2 || ids[2] == nodeId2;
        if (hasNode1 && hasNode2)
        {
            badFaces_.erase(it->first);
            it = restrictedFaces_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void RestrictedTriangulation::updateBadFaceEntry(const FaceKey& face,
                                                  const std::string& surfaceId,
                                                  const MeshData3D& meshData,
                                                  const Geometry3D::GeometryCollection3D& geometry)
{
    const RCDTQualityController qualityController(meshData, geometry, settings_);
    const TriangleElement triangle(face.nodeIds);

    if (qualityController.isTriangleAcceptable(triangle, surfaceId))
    {
        badFaces_.erase(face);
        return;
    }

    const ElementGeometry3D elementGeometry(meshData);
    const auto circumcircle = elementGeometry.computeCircumcircle(triangle);
    if (!circumcircle)
    {
        badFaces_.erase(face);
        return;
    }

    const ElementQuality3D elementQuality(meshData);
    const double shortestEdge = elementQuality.getShortestEdgeLength(triangle);
    badFaces_.insert_or_assign(face, BadRestrictedTriangle{face, surfaceId, circumcircle->center, shortestEdge});
}

std::vector<BadRestrictedTriangle> RestrictedTriangulation::getBadTriangles() const
{
    if (restrictedFaces_.size() >= settings_.elementLimit)
        return {};

    std::vector<BadRestrictedTriangle> badTriangles;
    badTriangles.reserve(badFaces_.size());
    for (const auto& [face, badTriangle] : badFaces_)
        badTriangles.push_back(badTriangle);
    return badTriangles;
}

const std::unordered_map<FaceKey, std::string, FaceKeyHash>& RestrictedTriangulation::getRestrictedFaces() const
{
    return restrictedFaces_;
}

size_t RestrictedTriangulation::removeChordFaces(const MeshData3D& meshData)
{
    std::vector<FaceKey> candidates;
    for (const auto& [face, surfaceId] : restrictedFaces_)
        if (hasSameCurveChordEdge(face, meshData))
            candidates.push_back(face);

    if (candidates.empty())
        return 0;

    std::unordered_map<EdgeKey, int, EdgeKeyHash> edgeCounts;
    for (const auto& [face, surfaceId] : restrictedFaces_)
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
                if (isSameCurveChordEdge(nodeIdA, nodeIdB, meshData))
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
            restrictedFaces_.erase(face);
            badFaces_.erase(face);
            ++removed;
            it = candidates.erase(it);
            progress = true;
        }
    }
    return removed;
}

std::optional<Point3D> RestrictedTriangulation::computeInsertionPoint(
    const FaceKey& face,
    const MeshData3D& meshData,
    const MeshConnectivity& connectivity,
    const Geometry3D::ISurface3D& surface) const
{
    const auto endpoints = computeDualEdgeEndpoints(face, meshData, connectivity);
    if (!endpoints)
        return std::nullopt;
    return surfaceProjector_.findSurfaceCrossing(endpoints->first, endpoints->second, surface);
}

std::vector<NonManifoldRestrictedEdge> RestrictedTriangulation::findNonManifoldEdges() const
{
    std::unordered_map<EdgeKey, int, EdgeKeyHash> edgeCounts;
    std::unordered_map<EdgeKey, std::string, EdgeKeyHash> edgeSurfaceIds;

    for (const auto& [face, surfaceId] : restrictedFaces_)
    {
        for (const auto& edge : faceEdges(face))
        {
            ++edgeCounts[edge];
            edgeSurfaceIds.try_emplace(edge, surfaceId);
        }
    }

    std::vector<NonManifoldRestrictedEdge> nonManifoldEdges;
    for (const auto& [edge, count] : edgeCounts)
    {
        if (count != 2)
            nonManifoldEdges.push_back({edge, edgeSurfaceIds.at(edge)});
    }
    return nonManifoldEdges;
}

std::optional<std::string> RestrictedTriangulation::classifyFace(const FaceKey& face,
                                                                 const MeshData3D& meshData,
                                                                 const MeshConnectivity& connectivity,
                                                                 const Geometry3D::GeometryCollection3D& geometry) const
{
    // NOTE: This is a fragile function.

    // Intersect effective surface IDs across all 3 nodes to find candidate surfaces.
    std::unordered_set<std::string> candidates;
    bool firstNode = true;
    for (const size_t nodeId : face.nodeIds)
    {
        if (!meshData.getNode(nodeId))
            return std::nullopt;

        const auto nodeSurfaces = effectiveSurfaceIds(meshData.getGeometryIds(nodeId));
        if (firstNode)
        {
            candidates = nodeSurfaces;
            firstNode = false;
        }
        else
        {
            for (auto it = candidates.begin(); it != candidates.end();)
            {
                if (!nodeSurfaces.count(*it))
                    it = candidates.erase(it);
                else
                    ++it;
            }
        }

        if (candidates.empty())
            return std::nullopt;
    }

    const auto& [elementId1, elementId2] = connectivity.getFaceElements(face);
    if (elementId1 == INVALID_ID)
        return std::nullopt;

    // Convex-hull (boundary) face: the dual Voronoi edge is a half-infinite ray
    // from the one adjacent tet's circumcenter outward. It always crosses the
    // surface when the circumcenter is on the interior side, which is
    // guaranteed for Delaunay triangulations of boundary points on a closed
    // solid.
    if (elementId2 == INVALID_ID)
    {
        const auto* tet1 = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId1));
        if (!tet1)
            return std::nullopt;

        const ElementGeometry3D elementGeometry(meshData);
        if (!elementGeometry.computeCircumscribingSphere(*tet1))
            return std::nullopt;

        for (const auto& surfaceId : candidates)
        {
            const Geometry3D::ISurface3D* surface = geometry.getSurface(surfaceId);
            if (surface && verticesWithinTrimmedBoundary(face, meshData, *surface))
                return surfaceId;
        }
        return std::nullopt;
    }

    const auto endpoints = computeDualEdgeEndpoints(face, meshData, connectivity);
    if (!endpoints)
        return std::nullopt;

    // A face carrying a genuinely protected edge (two of its three nodes
    // chain-adjacent along the same curve) has a structural guarantee
    // crossesSurface()'s dual-edge oracle doesn't: property 1's overlapping
    // protecting balls certify that edge belongs to exactly one crease. But
    // that only certifies the EDGE, not this particular FACE -- an edge in a
    // tetrahedralization is generally shared by many faces (the ring of tets
    // surrounding it), not just the two genuine boundary ones, so trusting
    // any single-candidate face touching a protected edge is unsound (tried
    // and reverted, see OPE-176 project memory: it accepted spurious faces
    // from elsewhere in that ring). Requiring this face to be the UNIQUE
    // candidate across the whole edge star narrows the shortcut to the
    // specific failure this is meant to fix: crossesSurface()'s
    // near-degenerate crossing test coming back negative for the sole
    // legitimate candidate.
    if (candidates.size() == 1)
    {
        const std::string& surfaceId = *candidates.begin();
        const Geometry3D::ISurface3D* surface = geometry.getSurface(surfaceId);
        if (surface && verticesWithinTrimmedBoundary(face, meshData, *surface))
        {
            if (const auto protectedEdge = findProtectedEdge(face, meshData))
            {
                if (isUniqueEdgeStarCandidate(face, protectedEdge->first, protectedEdge->second, surfaceId, *surface,
                                              meshData, connectivity))
                {
                    return surfaceId;
                }
            }

            // Skipped for periodic (seam) surfaces -- see periodicSurfaceIds_'s
            // member doc: unlike an ordinary crease, a seam surface exhibits a
            // small but persistent stream of misclassifications through this
            // path that prevents refinement from ever converging.
            if (!periodicSurfaceIds_.count(surfaceId) && isPhaseBoundaryFace(face, meshData, connectivity, geometry))
            {
                const auto& n = face.nodeIds;
                const std::array<std::pair<size_t, size_t>, 3> edges = {
                    std::make_pair(n[0], n[1]), std::make_pair(n[0], n[2]), std::make_pair(n[1], n[2])};
                for (const auto& edge : edges)
                {
                    if (isUniquePhaseBoundaryCandidate(face, edge.first, edge.second, *surface, meshData,
                                                       connectivity, geometry))
                    {
                        return surfaceId;
                    }
                }
            }
        }
    }

    for (const auto& surfaceId : candidates)
    {
        const Geometry3D::ISurface3D* surface = geometry.getSurface(surfaceId);
        if (!surface)
            continue;
        if (!verticesWithinTrimmedBoundary(face, meshData, *surface))
            continue;
        const auto tessellationIt = surfaceTessellations_.find(surfaceId);
        if (tessellationIt == surfaceTessellations_.end())
            continue;
        if (tessellationIt->second.crossesSurface(endpoints->first, endpoints->second))
            return surfaceId;
    }

    return std::nullopt;
}

std::optional<std::pair<size_t, size_t>> RestrictedTriangulation::findProtectedEdge(const FaceKey& face,
                                                                                     const MeshData3D& meshData)
{
    const auto& curveSegmentManager = meshData.getCurveSegmentManager();
    const auto& n = face.nodeIds;
    const std::array<std::pair<size_t, size_t>, 3> edges = {
        std::make_pair(n[0], n[1]), std::make_pair(n[0], n[2]), std::make_pair(n[1], n[2])};
    for (const auto& edge : edges)
    {
        if (curveSegmentManager.findSegmentId(edge.first, edge.second))
            return edge;
    }
    return std::nullopt;
}

bool RestrictedTriangulation::isSameCurveChordEdge(size_t nodeIdA, size_t nodeIdB, const MeshData3D& meshData) const
{
    if (meshData.getCurveSegmentManager().findSegmentId(nodeIdA, nodeIdB))
        return false; // chain-adjacent -- a genuine protected edge, not a chord

    const auto& idsB = meshData.getGeometryIds(nodeIdB);
    for (const auto& geometryId : meshData.getGeometryIds(nodeIdA))
    {
        if (!edgeToAdjacentSurfaces_.count(geometryId))
            continue; // not an edge (curve)-type geometryId -- a surface or corner tag
        if (std::find(idsB.begin(), idsB.end(), geometryId) != idsB.end())
            return true; // both endpoints on the same curve, but not chain-adjacent
    }
    return false;
}

bool RestrictedTriangulation::hasSameCurveChordEdge(const FaceKey& face, const MeshData3D& meshData) const
{
    const auto& n = face.nodeIds;
    return isSameCurveChordEdge(n[0], n[1], meshData) || isSameCurveChordEdge(n[0], n[2], meshData) ||
           isSameCurveChordEdge(n[1], n[2], meshData);
}

bool RestrictedTriangulation::isUniqueEdgeStarCandidate(const FaceKey& face,
                                                        size_t nodeIdA,
                                                        size_t nodeIdB,
                                                        const std::string& surfaceId,
                                                        const Geometry3D::ISurface3D& surface,
                                                        const MeshData3D& meshData,
                                                        const MeshConnectivity& connectivity) const
{
    // verticesWithinTrimmedBoundary alone is NOT a local test -- it passes
    // for any point actually on surfaceId's CAD patch, however far from this
    // specific edge, so most crease edges have plenty of vertex-in-trim
    // "rivals" purely from unrelated points elsewhere on the same surface
    // (confirmed on SaddleSurfaceMesh: >95% of protected-edge candidates
    // rejected as ambiguous by that test alone, with no measurable effect on
    // the final mesh -- see OPE-176 project memory). crossesSurface() is a
    // genuinely local test (it's testing THIS rival's own dual Voronoi edge),
    // and it's only unreliable in the near-degenerate case this shortcut
    // exists to work around -- which happens right at the true crease, not
    // several tets away at an unrelated rival -- so requiring a rival to
    // also pass ITS OWN crossesSurface() before it counts as a genuine
    // competitor keeps this check local without reintroducing the oracle's
    // failure mode for our own face.
    const auto tessellationIt = surfaceTessellations_.find(surfaceId);
    if (tessellationIt == surfaceTessellations_.end())
        return false;

    std::unordered_set<FaceKey, FaceKeyHash> edgeStar;
    for (const size_t elementId : connectivity.getNodeElements(nodeIdA))
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId));
        if (!tet)
            continue;

        const auto& tetNodeIds = tet->getNodeIds();
        const bool touchesB = std::find(tetNodeIds.begin(), tetNodeIds.end(), nodeIdB) != tetNodeIds.end();
        if (!touchesB)
            continue;

        for (const auto& faceArray : tet->getFaces())
        {
            const FaceKey candidateFace(faceArray);
            const auto& ids = candidateFace.nodeIds;
            const bool hasA = ids[0] == nodeIdA || ids[1] == nodeIdA || ids[2] == nodeIdA;
            const bool hasB = ids[0] == nodeIdB || ids[1] == nodeIdB || ids[2] == nodeIdB;
            if (hasA && hasB)
                edgeStar.insert(candidateFace);
        }
    }

    for (const auto& candidateFace : edgeStar)
    {
        if (candidateFace == face)
            continue;
        if (!verticesWithinTrimmedBoundary(candidateFace, meshData, surface))
            continue;
        const auto endpoints = computeDualEdgeEndpoints(candidateFace, meshData, connectivity);
        if (!endpoints)
            continue;
        if (tessellationIt->second.crossesSurface(endpoints->first, endpoints->second))
            return false;
    }
    return true;
}

bool RestrictedTriangulation::isPhaseBoundaryFace(const FaceKey& face,
                                                   const MeshData3D& meshData,
                                                   const MeshConnectivity& connectivity,
                                                   const Geometry3D::GeometryCollection3D& geometry) const
{
    if (volumeIds_.empty())
        return false;

    const auto& [elementId1, elementId2] = connectivity.getFaceElements(face);
    if (elementId1 == INVALID_ID || elementId2 == INVALID_ID)
        return false;

    const auto* tet1 = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId1));
    const auto* tet2 = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId2));
    if (!tet1 || !tet2)
        return false;

    const ElementGeometry3D elementGeometry(meshData);
    const PointPhase phase1 = classifyPointPhase(elementGeometry.computeCentroid(*tet1), volumeIds_, geometry);
    const PointPhase phase2 = classifyPointPhase(elementGeometry.computeCentroid(*tet2), volumeIds_, geometry);
    if (phase1.kind == PointPhaseKind::Ambiguous || phase2.kind == PointPhaseKind::Ambiguous)
        return false;

    if (phase1.kind != phase2.kind)
        return true;

    return phase1.kind == PointPhaseKind::InVolume && phase1.volumeId != phase2.volumeId;
}

bool RestrictedTriangulation::isUniquePhaseBoundaryCandidate(const FaceKey& face,
                                                              size_t nodeIdA,
                                                              size_t nodeIdB,
                                                              const Geometry3D::ISurface3D& surface,
                                                              const MeshData3D& meshData,
                                                              const MeshConnectivity& connectivity,
                                                              const Geometry3D::GeometryCollection3D& geometry) const
{
    std::unordered_set<FaceKey, FaceKeyHash> edgeStar;
    for (const size_t elementId : connectivity.getNodeElements(nodeIdA))
    {
        const auto* tet = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId));
        if (!tet)
            continue;

        const auto& tetNodeIds = tet->getNodeIds();
        const bool touchesB = std::find(tetNodeIds.begin(), tetNodeIds.end(), nodeIdB) != tetNodeIds.end();
        if (!touchesB)
            continue;

        for (const auto& faceArray : tet->getFaces())
        {
            const FaceKey candidateFace(faceArray);
            const auto& ids = candidateFace.nodeIds;
            const bool hasA = ids[0] == nodeIdA || ids[1] == nodeIdA || ids[2] == nodeIdA;
            const bool hasB = ids[0] == nodeIdB || ids[1] == nodeIdB || ids[2] == nodeIdB;
            if (hasA && hasB)
                edgeStar.insert(candidateFace);
        }
    }

    int rivalCount = 0;
    for (const auto& candidateFace : edgeStar)
    {
        if (candidateFace == face)
            continue;
        if (!verticesWithinTrimmedBoundary(candidateFace, meshData, surface))
            continue;
        if (isPhaseBoundaryFace(candidateFace, meshData, connectivity, geometry))
        {
            ++rivalCount;
            if (rivalCount > 1)
                return false;
        }
    }
    return true;
}

std::optional<size_t> RestrictedTriangulation::findTouchedBoundingNode(size_t tetId, const MeshData3D& meshData)
{
    const auto& boundingNodeIds = meshData.getBoundingNodeIds();
    if (!boundingNodeIds)
        return std::nullopt;

    const auto* tet = dynamic_cast<const TetrahedralElement*>(meshData.getElement(tetId));
    if (!tet)
        return std::nullopt;

    for (const size_t nodeId : tet->getNodeIds())
        for (const size_t boundingId : *boundingNodeIds)
            if (nodeId == boundingId)
                return boundingId;
    return std::nullopt;
}

std::optional<Point3D> RestrictedTriangulation::computeDualEdgeEndpoint(size_t tetId, const MeshData3D& meshData) const
{
    const auto* tet = dynamic_cast<const TetrahedralElement*>(meshData.getElement(tetId));
    if (!tet)
        return std::nullopt;

    // The real circumcenter is preferred even for a tet touching a bounding
    // supertet node (a deliberately huge seed-triangulation artifact -- see
    // Delaunay3D's class docs): SurfaceTessellation's crossing test is exact
    // regardless of how large or skewed the coordinates are, so an
    // extreme-but-computable circumcenter is fine, and staying close to the
    // tet's actual location keeps the crossing point local to the face being
    // classified. Substituting the bounding node's own (very far, laterally
    // arbitrary) coordinates instead can shift a segment's true crossing
    // point on a bounded tessellation well away from the face it's supposed
    // to be testing (see OPE-169) -- so that substitution is now only a
    // fallback for when the circumsphere solve genuinely fails to converge.
    const ElementGeometry3D elementGeometry(meshData);
    if (const auto sphere = elementGeometry.computeCircumscribingSphere(*tet))
        return sphere->center;

    if (const auto boundingId = findTouchedBoundingNode(tetId, meshData))
    {
        if (const Node3D* boundingNode = meshData.getNode(*boundingId))
            return boundingNode->getCoordinates();
    }

    return std::nullopt;
}

std::optional<std::pair<Point3D, Point3D>> RestrictedTriangulation::computeDualEdgeEndpoints(
    const FaceKey& face,
    const MeshData3D& meshData,
    const MeshConnectivity& connectivity) const
{
    const auto& [elementId1, elementId2] = connectivity.getFaceElements(face);
    if (elementId1 == INVALID_ID || elementId2 == INVALID_ID)
        return std::nullopt;

    const auto endpoint1 = computeDualEdgeEndpoint(elementId1, meshData);
    const auto endpoint2 = computeDualEdgeEndpoint(elementId2, meshData);
    if (!endpoint1 || !endpoint2)
        return std::nullopt;

    return std::make_pair(*endpoint1, *endpoint2);
}

bool RestrictedTriangulation::verticesWithinTrimmedBoundary(const FaceKey& face,
                                                             const MeshData3D& meshData,
                                                             const Geometry3D::ISurface3D& surface) const
{
    for (const size_t nodeId : face.nodeIds)
    {
        const Node3D* node = meshData.getNode(nodeId);
        if (!node)
            return false;
        // Project the node's 3D coordinates to UV, then classify in UV space.
        // Avoids the 3D-point overload of BRepClass_FaceClassifier, which
        // internally triggers Extrema_GenExtPS (a grid-based surface projection
        // that rebuilds its grid on every call for Bezier/NURBS surfaces).
        // projectPointToUnderlyingSurface uses the cached ShapeAnalysis_Surface
        // analyzer (O(grid_scan + Newton) ≈ a few microseconds per call after
        // the first), while isUVWithinTrimmedBoundary uses OCC's 2D face
        // classifier (O(numEdges), always fast).
        const auto uv = surface.projectPointToUnderlyingSurface(node->getCoordinates());
        if (!uv || !surface.isUVWithinTrimmedBoundary(uv->x(), uv->y()))
            return false;
    }
    return true;
}

std::unordered_set<std::string> RestrictedTriangulation::effectiveSurfaceIds(
    const std::vector<std::string>& geometryIds) const
{
    std::unordered_set<std::string> result;
    for (const auto& id : geometryIds)
    {
        if (surfaceIds_.count(id))
        {
            result.insert(id);
        }
        else
        {
            const auto edgeIt = edgeToAdjacentSurfaces_.find(id);
            if (edgeIt != edgeToAdjacentSurfaces_.end())
            {
                for (const auto& surfaceId : edgeIt->second)
                    result.insert(surfaceId);
            }
            else
            {
                const auto cornerIt = cornerToAdjacentSurfaces_.find(id);
                if (cornerIt != cornerToAdjacentSurfaces_.end())
                    for (const auto& surfaceId : cornerIt->second)
                        result.insert(surfaceId);
            }
        }
    }
    return result;
}

} // namespace Meshing
