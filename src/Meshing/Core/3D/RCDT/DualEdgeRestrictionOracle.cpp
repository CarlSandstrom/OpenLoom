#include "Meshing/Core/3D/RCDT/DualEdgeRestrictionOracle.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Geometry/3D/Base/IVolume3D.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/RCDT/SurfaceCandidates.h"
#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "Topology/Topology3D.h"

#include <algorithm>
#include <array>

namespace Meshing
{

namespace
{

constexpr size_t INVALID_ID = SIZE_MAX;

// The tessellation oracle's target cell size is this fraction of minimumEdgeLength.
// Cells smaller than minimumEdgeLength / 2 are guaranteed fine enough to
// correctly classify any face whose shortest edge is at or above that floor.
constexpr double TESSELLATION_CELL_SIZE_FACTOR = 0.5;

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

DualEdgeRestrictionOracle::DualEdgeRestrictionOracle(const Geometry3D::GeometryCollection3D& geometry,
                                                     const Topology3D::Topology3D& topology,
                                                     const SurfaceCandidates& surfaceCandidates,
                                                     double minimumEdgeLength) :
    surfaceCandidates_(&surfaceCandidates),
    volumeIds_(topology.getAllVolumeIds())
{
    for (const auto& surfaceId : surfaceCandidates_->getSurfaceIds())
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

    const double targetCellSize = minimumEdgeLength * TESSELLATION_CELL_SIZE_FACTOR;
    for (const auto& surfaceId : surfaceCandidates_->getSurfaceIds())
    {
        const Geometry3D::ISurface3D* surface = geometry.getSurface(surfaceId);
        if (!surface)
            continue;
        surfaceTessellations_[surfaceId].build(*surface, targetCellSize);
    }
}

std::optional<Point3D> DualEdgeRestrictionOracle::insertionPointFor(
    const FaceKey& face,
    const MeshData3D& meshData,
    const MeshConnectivity& connectivity,
    const Geometry3D::ISurface3D& surface) const
{
    const auto endpoints = computeDualEdgeEndpoints(face, meshData, connectivity);
    if (!endpoints)
        return std::nullopt;
    const SurfaceProjector surfaceProjector;
    return surfaceProjector.findSurfaceCrossing(endpoints->first, endpoints->second, surface);
}

FaceClassification DualEdgeRestrictionOracle::classify(const FaceKey& face,
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
            return {FaceRestriction::NotRestricted, {}};

        const auto nodeSurfaces = surfaceCandidates_->effectiveSurfaceIds(meshData.getGeometryIds(nodeId));
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
            return {FaceRestriction::NotRestricted, {}};
    }

    // No adjacent element at all: not a face of the tetrahedralization, so
    // there is nothing here that could have been missed.
    const auto& [elementId1, elementId2] = connectivity.getFaceElements(face);
    if (elementId1 == INVALID_ID)
        return {FaceRestriction::NotRestricted, {}};

    // Convex-hull (boundary) face: the dual Voronoi edge is a half-infinite ray
    // from the one adjacent tet's circumcenter outward. It always crosses the
    // surface when the circumcenter is on the interior side, which is
    // guaranteed for Delaunay triangulations of boundary points on a closed
    // solid.
    if (elementId2 == INVALID_ID)
    {
        const auto* tet1 = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId1));
        if (!tet1)
            return {FaceRestriction::NotRestricted, {}};

        const ElementGeometry3D elementGeometry(meshData);
        if (!elementGeometry.computeCircumscribingSphere(*tet1))
            return {FaceRestriction::Unconfirmed, {}};

        for (const auto& surfaceId : candidates)
        {
            const Geometry3D::ISurface3D* surface = geometry.getSurface(surfaceId);
            if (surface && verticesWithinTrimmedBoundary(face, surfaceId, meshData, *surface))
                return {FaceRestriction::Restricted, surfaceId};
        }
        return {FaceRestriction::Unconfirmed, {}};
    }

    const auto endpoints = computeDualEdgeEndpoints(face, meshData, connectivity);
    if (!endpoints)
        return {FaceRestriction::Unconfirmed, {}};

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
        if (surface && verticesWithinTrimmedBoundary(face, surfaceId, meshData, *surface))
        {
            if (const auto protectedEdge = findProtectedEdge(face, meshData))
            {
                if (isUniqueEdgeStarCandidate(face, protectedEdge->first, protectedEdge->second, surfaceId, *surface,
                                              meshData, connectivity))
                {
                    return {FaceRestriction::Restricted, surfaceId};
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
                    if (isUniquePhaseBoundaryCandidate(face, edge.first, edge.second, surfaceId, *surface,
                                                       meshData, connectivity, geometry))
                    {
                        return {FaceRestriction::Restricted, surfaceId};
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
        if (!verticesWithinTrimmedBoundary(face, surfaceId, meshData, *surface))
            continue;
        const auto tessellationIt = surfaceTessellations_.find(surfaceId);
        if (tessellationIt == surfaceTessellations_.end())
            continue;
        if (tessellationIt->second.crossesSurface(endpoints->first, endpoints->second))
            return {FaceRestriction::Restricted, surfaceId};
    }

    return {FaceRestriction::Unconfirmed, {}};
}

std::optional<std::pair<size_t, size_t>> DualEdgeRestrictionOracle::findProtectedEdge(const FaceKey& face,
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

bool DualEdgeRestrictionOracle::isUniqueEdgeStarCandidate(const FaceKey& face,
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
        if (!verticesWithinTrimmedBoundary(candidateFace, surfaceId, meshData, surface))
            continue;
        const auto endpoints = computeDualEdgeEndpoints(candidateFace, meshData, connectivity);
        if (!endpoints)
            continue;
        if (tessellationIt->second.crossesSurface(endpoints->first, endpoints->second))
            return false;
    }
    return true;
}

const PointPhase& DualEdgeRestrictionOracle::centroidPhase(
    const TetrahedralElement& tetrahedron,
    const MeshData3D& meshData,
    const Geometry3D::GeometryCollection3D& geometry) const
{
    const auto& nodeIds = tetrahedron.getNodeIds();
    const TetrahedronKey key(nodeIds[0], nodeIds[1], nodeIds[2], nodeIds[3]);

    const auto found = centroidPhaseByTetrahedron_.find(key);
    if (found != centroidPhaseByTetrahedron_.end())
        return found->second;

    const ElementGeometry3D elementGeometry(meshData);
    return centroidPhaseByTetrahedron_
        .emplace(key, classifyPointPhase(elementGeometry.computeCentroid(tetrahedron), volumeIds_, geometry))
        .first->second;
}

bool DualEdgeRestrictionOracle::isPhaseBoundaryFace(const FaceKey& face,
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

    const PointPhase& phase1 = centroidPhase(*tet1, meshData, geometry);
    const PointPhase& phase2 = centroidPhase(*tet2, meshData, geometry);
    if (phase1.kind == PointPhaseKind::Ambiguous || phase2.kind == PointPhaseKind::Ambiguous)
        return false;

    if (phase1.kind != phase2.kind)
        return true;

    return phase1.kind == PointPhaseKind::InVolume && phase1.volumeId != phase2.volumeId;
}

bool DualEdgeRestrictionOracle::isUniquePhaseBoundaryCandidate(const FaceKey& face,
                                                               size_t nodeIdA,
                                                               size_t nodeIdB,
                                                               const std::string& surfaceId,
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
        if (!verticesWithinTrimmedBoundary(candidateFace, surfaceId, meshData, surface))
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

std::optional<size_t> DualEdgeRestrictionOracle::findTouchedBoundingNode(size_t tetId, const MeshData3D& meshData)
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

std::optional<Point3D> DualEdgeRestrictionOracle::computeDualEdgeEndpoint(size_t tetId, const MeshData3D& meshData) const
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

std::optional<std::pair<Point3D, Point3D>> DualEdgeRestrictionOracle::computeDualEdgeEndpoints(
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

bool DualEdgeRestrictionOracle::verticesWithinTrimmedBoundary(const FaceKey& face,
                                                              const std::string& surfaceId,
                                                              const MeshData3D& meshData,
                                                              const Geometry3D::ISurface3D& surface) const
{
    for (const size_t nodeId : face.nodeIds)
    {
        const Node3D* node = meshData.getNode(nodeId);
        if (!node)
            return false;
        if (!nodeWithinTrimmedBoundary(surfaceId, nodeId, node->getCoordinates(), surface))
            return false;
    }
    return true;
}

bool DualEdgeRestrictionOracle::nodeWithinTrimmedBoundary(const std::string& surfaceId,
                                                          size_t nodeId,
                                                          const Point3D& coordinates,
                                                          const Geometry3D::ISurface3D& surface) const
{
    auto& cacheForSurface = nodeWithinTrimmedBoundaryBySurface_[surfaceId];
    const auto found = cacheForSurface.find(nodeId);
    if (found != cacheForSurface.end())
        return found->second;

    // Project the node's 3D coordinates to UV, then classify in UV space.
    // Avoids the 3D-point overload of BRepClass_FaceClassifier, which
    // internally triggers Extrema_GenExtPS (a grid-based surface projection
    // that rebuilds its grid on every call for Bezier/NURBS surfaces).
    // isUVWithinTrimmedBoundary then uses OCC's 2D face classifier
    // (O(numEdges), always fast, and cached per surface).
    //
    // projectPointToUnderlyingSurface is NOT cheap despite going through the
    // cached ShapeAnalysis_Surface analyzer -- ValueOfUV still runs a global
    // Extrema search per call -- which is why the result is memoized here
    // rather than recomputed per face.
    const auto uv = surface.projectPointToUnderlyingSurface(coordinates);
    const bool within = uv.has_value() && surface.isUVWithinTrimmedBoundary(uv->x(), uv->y());
    return cacheForSurface.emplace(nodeId, within).first->second;
}

} // namespace Meshing
