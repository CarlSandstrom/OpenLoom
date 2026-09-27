#include "Meshing/Core/3D/RCDT/WeightedDualRestriction.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Topology/Topology3D.h"

#include <algorithm>
#include <cstdint>

namespace Meshing
{

namespace
{

constexpr size_t INVALID_ID = SIZE_MAX;

// Same resolution rule as DualEdgeRestrictionOracle: cells of half the
// minimum edge length resolve any face at or above that floor.
constexpr double TESSELLATION_CELL_SIZE_FACTOR = 0.5;

bool touchesBoundingTetrahedron(const FaceKey& face, const MeshData3D& meshData)
{
    const auto& boundingNodeIds = meshData.getBoundingNodeIds();
    if (!boundingNodeIds)
        return false;
    for (const size_t nodeId : face.nodeIds)
        if (std::find(boundingNodeIds->begin(), boundingNodeIds->end(), nodeId) != boundingNodeIds->end())
            return true;
    return false;
}

} // namespace

WeightedDualRestriction::WeightedDualRestriction(const Geometry3D::GeometryCollection3D& geometry,
                                                 const Topology3D::Topology3D& topology,
                                                 double minimumEdgeLength) :
    geometry_(&geometry)
{
    const double targetCellSize = minimumEdgeLength * TESSELLATION_CELL_SIZE_FACTOR;
    for (const auto& surfaceId : topology.getAllSurfaceIds())
    {
        const Geometry3D::ISurface3D* surface = geometry.getSurface(surfaceId);
        if (surface)
            surfaceTessellations_[surfaceId].build(*surface, targetCellSize);
    }
}

std::optional<RestrictedFacet> WeightedDualRestriction::restrict(const FaceKey& face,
                                                                 const MeshData3D& meshData,
                                                                 const MeshConnectivity& connectivity) const
{
    if (touchesBoundingTetrahedron(face, meshData))
        return std::nullopt;

    const auto& [elementId1, elementId2] = connectivity.getFaceElements(face);
    if (elementId1 == INVALID_ID || elementId2 == INVALID_ID)
        return std::nullopt;
    const auto* tetrahedron1 = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId1));
    const auto* tetrahedron2 = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId2));
    if (!tetrahedron1 || !tetrahedron2)
        return std::nullopt;

    const ElementGeometry3D elementGeometry(meshData);
    const auto dualStart = elementGeometry.computeOrthocenter(*tetrahedron1);
    const auto dualEnd = elementGeometry.computeOrthocenter(*tetrahedron2);
    const auto faceCenter = elementGeometry.computeOrthocenter(TriangleElement(face.nodeIds));
    if (!dualStart || !dualEnd || !faceCenter)
        return std::nullopt;

    std::optional<RestrictedFacet> nearest;
    double nearestSquaredDistance = 0.0;
    for (const auto& [surfaceId, tessellation] : surfaceTessellations_)
    {
        const auto crossing = tessellation.findCrossingNearest(*dualStart, *dualEnd, *faceCenter);
        if (!crossing)
            continue;
        const double squaredDistance = (*crossing - *faceCenter).squaredNorm();
        if (!nearest || squaredDistance < nearestSquaredDistance)
        {
            nearest = RestrictedFacet{surfaceId, *crossing};
            nearestSquaredDistance = squaredDistance;
        }
    }
    if (!nearest)
        return std::nullopt;

    // The tessellation approximates the surface; the centre is used as an
    // insertion point, so it belongs on the surface itself.
    if (const auto onSurface =
            SurfaceProjector::projectToSurface(nearest->surfaceCenter, *geometry_->getSurface(nearest->surfaceId)))
        nearest->surfaceCenter = *onSurface;
    return nearest;
}

} // namespace Meshing
