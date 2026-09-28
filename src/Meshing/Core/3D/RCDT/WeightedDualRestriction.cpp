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
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace Meshing
{

namespace
{

constexpr size_t INVALID_ID = SIZE_MAX;

// Cells of half the minimum edge length resolve any face at or above that
// floor.
constexpr double TESSELLATION_CELL_SIZE_FACTOR = 0.5;

// Half-width, in tessellation cells, of the bracket along the dual segment
// in which the exact crossing is sought around the tessellation's.
constexpr double BRACKET_CELLS = 2.0;

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

/// The exact crossing of the dual segment with surface near the tessellation's
/// crossing, found by bisection along the segment itself in a bracket of a few
/// tessellation cells: the centre must stay on the dual line. Projecting onto
/// the surface instead moved it sideways, far enough to land inside a
/// protecting ball (HexNutChamfered, 0.035 off the line), which broke the
/// triangulation. Falls back to the tessellation's crossing when the bracket
/// has no sign change.
Point3D refineAlongDualSegment(const Point3D& tessellationCrossing,
                               const Point3D& dualStart,
                               const Point3D& dualEnd,
                               double cellSize,
                               const Geometry3D::ISurface3D& surface)
{
    const Point3D direction = dualEnd - dualStart;
    const double squaredLength = direction.squaredNorm();
    if (squaredLength <= 0.0)
        return tessellationCrossing;
    const double hit = (tessellationCrossing - dualStart).dot(direction) / squaredLength;
    const double halfWidth = BRACKET_CELLS * cellSize / std::sqrt(squaredLength);
    const Point3D bracketStart = dualStart + std::max(0.0, hit - halfWidth) * direction;
    const Point3D bracketEnd = dualStart + std::min(1.0, hit + halfWidth) * direction;
    return SurfaceProjector::findSurfaceCrossing(bracketStart, bracketEnd, surface).value_or(tessellationCrossing);
}

/// Whether point lies within surface's trimmed patch. The tessellation
/// extends up to a cell past the trim boundary, and the bisection above works
/// on the untrimmed surface, so a crossing can land on the surface but off the
/// face -- on SharpCreaseBracket, where two flange planes extended past their
/// faces intersect.
bool isWithinTrimmedPatch(const Point3D& point, const Geometry3D::ISurface3D& surface)
{
    const auto uv = surface.projectPointToUnderlyingSurface(point);
    return uv.has_value() && surface.isUVWithinTrimmedBoundary(uv->x(), uv->y());
}

} // namespace

WeightedDualRestriction::WeightedDualRestriction(const Geometry3D::GeometryCollection3D& geometry,
                                                 const Topology3D::Topology3D& topology,
                                                 double minimumEdgeLength) :
    geometry_(&geometry),
    cellSize_(minimumEdgeLength * TESSELLATION_CELL_SIZE_FACTOR)
{
    const double targetCellSize = cellSize_;
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

    // Each surface's crossing nearest the face, tried nearest first. The
    // first one lying within its surface's trimmed patch wins.
    std::vector<std::pair<double, RestrictedFacet>> crossings;
    for (const auto& [surfaceId, tessellation] : surfaceTessellations_)
        if (const auto crossing = tessellation.findCrossingNearest(*dualStart, *dualEnd, *faceCenter))
            crossings.push_back({(*crossing - *faceCenter).squaredNorm(), RestrictedFacet{surfaceId, *crossing}});
    std::sort(crossings.begin(), crossings.end(),
              [](const auto& a, const auto& b)
              { return a.first < b.first; });

    for (auto& [squaredDistance, facet] : crossings)
    {
        const Geometry3D::ISurface3D& surface = *geometry_->getSurface(facet.surfaceId);
        facet.surfaceCenter = refineAlongDualSegment(facet.surfaceCenter, *dualStart, *dualEnd, cellSize_, surface);
        if (isWithinTrimmedPatch(facet.surfaceCenter, surface))
            return facet;
    }
    return std::nullopt;
}

} // namespace Meshing
