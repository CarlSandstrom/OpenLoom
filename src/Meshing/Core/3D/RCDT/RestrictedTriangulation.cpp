#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/ElementQuality3D.h"
#include "Meshing/Core/3D/RCDT/RCDTQualityController.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Topology/Topology3D.h"

#include <iterator>

namespace Meshing
{

namespace
{

constexpr size_t INVALID_ID = SIZE_MAX;

// The tessellation oracle's target cell size is this fraction of minimumEdgeLength.
// Cells smaller than minimumEdgeLength / 2 are guaranteed fine enough to
// correctly classify any face whose shortest edge is at or above that floor.
constexpr double TESSELLATION_CELL_SIZE_FACTOR = 0.5;

} // namespace

void RestrictedTriangulation::buildFrom(const MeshData3D& meshData,
                                        const MeshConnectivity& connectivity,
                                        const Geometry3D::GeometryCollection3D& geometry,
                                        const Topology3D::Topology3D& topology,
                                        double minimumEdgeLength)
{
    restrictedFaces_.clear();
    surfaceIds_.clear();
    edgeToAdjacentSurfaces_.clear();
    surfaceTessellations_.clear();

    for (const auto& surfaceId : topology.getAllSurfaceIds())
        surfaceIds_.insert(surfaceId);

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
                restrictedFaces_.emplace(face, std::move(*surfaceId));
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
        restrictedFaces_.erase(face);

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
                restrictedFaces_.insert_or_assign(face, std::move(*surfaceId));
            else
                restrictedFaces_.erase(face);
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
        it = (hasNode1 && hasNode2) ? restrictedFaces_.erase(it) : std::next(it);
    }
}

std::vector<BadRestrictedTriangle> RestrictedTriangulation::getBadTriangles(
    const SurfaceMesh3DQualitySettings& settings,
    const MeshData3D& meshData,
    const MeshConnectivity& connectivity,
    const Geometry3D::GeometryCollection3D& geometry) const
{
    std::vector<BadRestrictedTriangle> badTriangles;

    const RCDTQualityController qualityController(meshData, geometry, settings);
    if (qualityController.isMeshAcceptable(restrictedFaces_.size()))
        return badTriangles;

    const ElementGeometry3D elementGeometry(meshData);
    const ElementQuality3D elementQuality(meshData);

    for (const auto& [face, surfaceId] : restrictedFaces_)
    {
        const TriangleElement triangle(face.nodeIds);

        if (qualityController.isTriangleAcceptable(triangle, surfaceId))
            continue;

        const auto circumcircle = elementGeometry.computeCircumcircle(triangle);
        if (!circumcircle)
            continue;

        const double shortestEdge = elementQuality.getShortestEdgeLength(triangle);

        badTriangles.push_back({face, surfaceId, circumcircle->center, shortestEdge});
    }

    return badTriangles;
}

const std::unordered_map<FaceKey, std::string, FaceKeyHash>& RestrictedTriangulation::getRestrictedFaces() const
{
    return restrictedFaces_;
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
        const auto& n = face.nodeIds;
        const std::array<EdgeKey, 3> edges = {EdgeKey(n[0], n[1]), EdgeKey(n[0], n[2]), EdgeKey(n[1], n[2])};
        for (const auto& edge : edges)
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
