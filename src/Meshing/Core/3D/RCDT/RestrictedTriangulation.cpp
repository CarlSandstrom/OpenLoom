#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/ElementQuality3D.h"
#include "Meshing/Core/3D/RCDT/DualEdgeRestrictionOracle.h"
#include "Meshing/Core/3D/RCDT/RCDTQualityController.h"
#include "Meshing/Core/3D/RCDT/RestrictedFaceAudit.h"
#include "Meshing/Core/3D/RCDT/SurfaceCandidates.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Topology/Topology3D.h"

#include <unordered_set>

namespace Meshing
{

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

    // Collected as a set rather than counted inline: a face is reached once
    // from each of its two adjacent tetrahedra, so incrementing per visit
    // would double-count every interior face.
    std::unordered_set<FaceKey, FaceKeyHash> unconfirmedFaces;

    surfaceCandidates_ = SurfaceCandidates(topology);
    oracle_ = DualEdgeRestrictionOracle(geometry, topology, surfaceCandidates_, minimumEdgeLength);

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

            auto classification = oracle_.classify(face, meshData, connectivity, geometry);
            if (classification.restriction == FaceRestriction::Restricted)
            {
                updateBadFaceEntry(face, classification.surfaceId, meshData, geometry);
                restrictedFaces_.emplace(face, std::move(classification.surfaceId));
            }
            else if (classification.restriction == FaceRestriction::Unconfirmed)
            {
                unconfirmedFaces.insert(face);
            }
        }
    }

    unconfirmedFaceCount_ = unconfirmedFaces.size();
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
            auto classification = oracle_.classify(face, meshData, connectivity, geometry);
            if (classification.restriction == FaceRestriction::Restricted)
            {
                updateBadFaceEntry(face, classification.surfaceId, meshData, geometry);
                restrictedFaces_.insert_or_assign(face, std::move(classification.surfaceId));
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

const RestrictedFaceMap& RestrictedTriangulation::getRestrictedFaces() const
{
    return restrictedFaces_;
}

DefectiveFaceRemovalSummary RestrictedTriangulation::removeDefectiveFaces(const MeshData3D& meshData)
{
    return RestrictedFaceAudit::removeDefectiveFaces(restrictedFaces_, badFaces_, surfaceCandidates_.getEdgeToAdjacentSurfaces(), meshData);
}

std::optional<Point3D> RestrictedTriangulation::computeInsertionPoint(
    const FaceKey& face,
    const MeshData3D& meshData,
    const MeshConnectivity& connectivity,
    const Geometry3D::ISurface3D& surface) const
{
    return oracle_.insertionPointFor(face, meshData, connectivity, surface);
}

std::vector<NonManifoldRestrictedEdge> RestrictedTriangulation::findNonManifoldEdges(
    const MeshData3D& meshData) const
{
    return RestrictedFaceAudit::findNonManifoldEdges(restrictedFaces_, surfaceCandidates_.getEdgeToAdjacentSurfaces(), meshData);
}

} // namespace Meshing
