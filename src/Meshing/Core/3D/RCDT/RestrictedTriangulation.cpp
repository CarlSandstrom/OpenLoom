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

} // namespace

void RestrictedTriangulation::buildFrom(const MeshData3D& meshData,
                                        const MeshConnectivity& connectivity,
                                        const Geometry3D::GeometryCollection3D& geometry,
                                        const Topology3D::Topology3D& topology)
{
    restrictedFaces_.clear();
    surfaceIds_.clear();
    edgeToAdjacentSurfaces_.clear();

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

        // The point to actually insert if this triangle gets refined: where
        // the face's dual Voronoi edge crosses the surface, not the flat
        // circumcircle center above (which is only used for the chord
        // deviation check — a property of the flat triangle itself).
        const Geometry3D::ISurface3D* surface = geometry.getSurface(surfaceId);
        std::optional<Point3D> insertionPoint;
        if (surface)
        {
            const auto endpoints = computeDualEdgeEndpoints(face, meshData, connectivity);
            if (endpoints)
                insertionPoint = surfaceProjector_.findSurfaceCrossing(endpoints->first, endpoints->second, *surface);
        }

        badTriangles.push_back({face, surfaceId, circumcircle->center, shortestEdge, insertionPoint});
    }

    return badTriangles;
}

const std::unordered_map<FaceKey, std::string, FaceKeyHash>& RestrictedTriangulation::getRestrictedFaces() const
{
    return restrictedFaces_;
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

        if (!candidates.empty())
            return *candidates.begin();
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
        if (surfaceProjector_.crossesSurface(endpoints->first, endpoints->second, *surface))
            return surfaceId;
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

    const auto* tet1 = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId1));
    const auto* tet2 = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId2));
    if (!tet1 || !tet2)
        return std::nullopt;

    const ElementGeometry3D elementGeometry(meshData);
    const auto sphere1 = elementGeometry.computeCircumscribingSphere(*tet1);
    const auto sphere2 = elementGeometry.computeCircumscribingSphere(*tet2);
    if (!sphere1 || !sphere2)
        return std::nullopt;

    return std::make_pair(sphere1->center, sphere2->center);
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
