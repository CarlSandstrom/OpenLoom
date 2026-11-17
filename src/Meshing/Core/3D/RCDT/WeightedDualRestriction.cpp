#include "Meshing/Core/3D/RCDT/WeightedDualRestriction.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/SurfaceProjector.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Meshing/Data/Base/TriangleElement.h"
#include "Topology/Topology3D.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace Meshing
{

namespace
{

// Cells of half the minimum edge length resolve any face at or above that floor.
constexpr double TESSELLATION_CELL_SIZE_FACTOR = 0.5;

// Half-width, in tessellation cells, of the bracket along the dual segment in
// which the exact crossing is sought around the tessellation's.
constexpr double BRACKET_CELLS = 2.0;

bool touchesBoundingTetrahedron(const FaceKey& face, const MeshData3D& meshData);
Point3D refineAlongDualSegment(const SurfaceTessellation::Crossing& tessellationCrossing,
                               const Point3D& dualStart,
                               const Point3D& dualEnd,
                               double cellSize,
                               const Geometry3D::ISurface3D& surface);
bool isWithinTrimmedPatch(const Point3D& point, const Geometry3D::ISurface3D& surface);

} // namespace

WeightedDualRestriction::WeightedDualRestriction(const Geometry3D::GeometryCollection3D& geometry,
                                                 const Topology3D::Topology3D& topology,
                                                 double minimumEdgeLength) :
    geometry_(&geometry),
    cellSize_(minimumEdgeLength * TESSELLATION_CELL_SIZE_FACTOR)
{
    for (auto& surfaceId : topology.getAllSurfaceIds())
    {
        auto surface = geometry.getSurface(surfaceId);
        if (surface) surfaceTessellations_[surfaceId].build(*surface, cellSize_);
    }
}

std::optional<RestrictedFacet> WeightedDualRestriction::restrict(const FaceKey& face,
                                                                 const MeshData3D& meshData,
                                                                 const MeshConnectivity& connectivity) const
{
    if (touchesBoundingTetrahedron(face, meshData)) return std::nullopt;

    auto [elementId1, elementId2] = connectivity.getFaceElements(face);
    if (elementId1 == ElementId::INVALID || elementId2 == ElementId::INVALID) return std::nullopt;

    auto tetrahedron1 = dynamic_cast<const TetrahedronElement*>(meshData.getElement(elementId1));
    auto tetrahedron2 = dynamic_cast<const TetrahedronElement*>(meshData.getElement(elementId2));
    if (!tetrahedron1 || !tetrahedron2) return std::nullopt;

    ElementGeometry3D elementGeometry(meshData);
    auto dualStart = elementGeometry.computeOrthocenter(*tetrahedron1);
    auto dualEnd = elementGeometry.computeOrthocenter(*tetrahedron2);
    auto faceCenter = elementGeometry.computeOrthocenter(TriangleElement(face.nodeIds));
    if (!dualStart || !dualEnd || !faceCenter) return std::nullopt;

    // Each surface's crossing nearest the face, tried nearest first. The first
    // one lying within its surface's trimmed patch wins.
    struct SurfaceCrossing
    {
        double squaredDistance;
        const std::string* surfaceId;
        SurfaceTessellation::Crossing crossing;
    };
    std::vector<SurfaceCrossing> crossings;
    for (auto& [surfaceId, tessellation] : surfaceTessellations_)
    {
        if (auto crossing = tessellation.findCrossingNearest(*dualStart, *dualEnd, *faceCenter))
        {
            crossings.push_back({(crossing->point - *faceCenter).squaredNorm(), &surfaceId, *crossing});
        }
    }
    std::sort(crossings.begin(), crossings.end(), [](const auto& left, const auto& right)
              { return left.squaredDistance < right.squaredDistance; });

    for (auto& candidate : crossings)
    {
        auto& surface = *geometry_->getSurface(*candidate.surfaceId);
        auto center = refineAlongDualSegment(candidate.crossing, *dualStart, *dualEnd, cellSize_, surface);
        if (isWithinTrimmedPatch(center, surface)) return RestrictedFacet{*candidate.surfaceId, center};
    }
    return std::nullopt;
}

namespace
{

bool touchesBoundingTetrahedron(const FaceKey& face, const MeshData3D& meshData)
{
    auto& boundingNodeIds = meshData.getBoundingNodeIds();
    if (!boundingNodeIds) return false;

    for (auto nodeId : face.nodeIds)
    {
        if (std::find(boundingNodeIds->begin(), boundingNodeIds->end(), nodeId) != boundingNodeIds->end()) return true;
    }
    return false;
}

// The exact crossing of the dual segment with the surface, sought in a bracket of a few
// tessellation cells around the tessellation's crossing and seeded from its parameters. The
// centre must stay on the dual line: projecting onto the surface instead moved it sideways
// into a protecting ball (HexNutChamfered, 0.035 off the line). Falls back to the
// tessellation's crossing when there is no exact crossing within the bracket.
Point3D refineAlongDualSegment(const SurfaceTessellation::Crossing& tessellationCrossing,
                               const Point3D& dualStart,
                               const Point3D& dualEnd,
                               double cellSize,
                               const Geometry3D::ISurface3D& surface)
{
    Point3D direction = dualEnd - dualStart;
    double squaredLength = direction.squaredNorm();
    if (squaredLength <= 0.0) return tessellationCrossing.point;

    double hit = (tessellationCrossing.point - dualStart).dot(direction) / squaredLength;
    double halfWidth = BRACKET_CELLS * cellSize / std::sqrt(squaredLength);
    Point3D bracketStart = dualStart + std::max(0.0, hit - halfWidth) * direction;
    Point3D bracketEnd = dualStart + std::min(1.0, hit + halfWidth) * direction;
    return SurfaceProjector::findSurfaceCrossing(bracketStart, bracketEnd, surface, tessellationCrossing.uv)
        .value_or(tessellationCrossing.point);
}

// The tessellation extends up to a cell past the trim boundary and the exact crossing is solved on the
// untrimmed surface, so a crossing can land on the surface but off the face (SharpCreaseBracket,
// where two flange planes extended past their faces intersect).
bool isWithinTrimmedPatch(const Point3D& point, const Geometry3D::ISurface3D& surface)
{
    auto uv = surface.projectPointToUnderlyingSurface(point);
    return uv.has_value() && surface.isUVWithinTrimmedBoundary(uv->x(), uv->y());
}

} // namespace

} // namespace Meshing
