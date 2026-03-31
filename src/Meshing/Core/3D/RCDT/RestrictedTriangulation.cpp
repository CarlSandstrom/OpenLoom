#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Topology/Topology3D.h"

#include <algorithm>
#include <cmath>
#include <limits>

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
            const FaceKey face(faceArray);
            if (restrictedFaces_.count(face))
                continue;

            auto surfaceId = classifyFace(face, meshData, connectivity, geometry);
            if (surfaceId)
                restrictedFaces_.emplace(face, std::move(*surfaceId));
        }
    }
}

std::vector<BadRestrictedTriangle> RestrictedTriangulation::getBadTriangles(
    const RCDTQualitySettings& settings,
    const MeshData3D& meshData,
    const Geometry3D::GeometryCollection3D& geometry) const
{
    std::vector<BadRestrictedTriangle> badTriangles;
    const ElementGeometry3D elementGeometry(meshData);

    for (const auto& [face, surfaceId] : restrictedFaces_)
    {
        const TriangleElement triangle(face.nodeIds);

        const auto circumcircle = elementGeometry.computeCircumcircle(triangle);
        if (!circumcircle)
            continue;

        double shortestEdge = std::numeric_limits<double>::max();
        for (size_t i = 0; i < 3; ++i)
        {
            const Point3D& p1 = meshData.getNode(face.nodeIds[i])->getCoordinates();
            const Point3D& p2 = meshData.getNode(face.nodeIds[(i + 1) % 3])->getCoordinates();
            shortestEdge = std::min(shortestEdge, (p2 - p1).norm());
        }

        const bool failsRatio = shortestEdge > 0.0 &&
            circumcircle->radius / shortestEdge > settings.maximumCircumradiusToShortestEdgeRatio;

        const Geometry3D::ISurface3D* surface = geometry.getSurface(surfaceId);
        const bool failsChordDeviation = surface &&
            std::abs(surfaceProjector_.signedDistance(circumcircle->center, *surface)) >
                settings.maximumChordDeviation;

        if (failsRatio || failsChordDeviation)
            badTriangles.push_back({face, surfaceId, circumcircle->center});
    }

    return badTriangles;
}

const std::unordered_map<FaceKey, std::string, FaceKeyHash>&
RestrictedTriangulation::getRestrictedFaces() const
{
    return restrictedFaces_;
}

std::optional<std::string> RestrictedTriangulation::classifyFace(
    const FaceKey& face,
    const MeshData3D& meshData,
    const MeshConnectivity& connectivity,
    const Geometry3D::GeometryCollection3D& geometry) const
{
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

    // Get the two adjacent tet circumcenters.
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

    for (const auto& surfaceId : candidates)
    {
        const Geometry3D::ISurface3D* surface = geometry.getSurface(surfaceId);
        if (!surface)
            continue;
        if (surfaceProjector_.crossesSurface(sphere1->center, sphere2->center, *surface))
            return surfaceId;
    }

    return std::nullopt;
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
            const auto it = edgeToAdjacentSurfaces_.find(id);
            if (it != edgeToAdjacentSurfaces_.end())
                for (const auto& surfaceId : it->second)
                    result.insert(surfaceId);
        }
    }
    return result;
}

} // namespace Meshing
