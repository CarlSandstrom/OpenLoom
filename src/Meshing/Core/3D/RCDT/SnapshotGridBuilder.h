#pragma once

#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/SurfaceFacetCriteria.h"
#include "Meshing/Core/3D/RCDT/WeightedDualRestriction.h"

#include <unordered_map>
#include <unordered_set>

namespace Export
{
struct VtkGrid;
} // namespace Export

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{

class MeshData3D;
struct SurfaceMesh3D;

/// Lays RCDT state out as the grids MeshSnapshotWriter writes.
class SnapshotGridBuilder
{
public:
    SnapshotGridBuilder() = delete;

    /// Every node as a point, every triangle as a cell, coloured by the
    /// SurfaceID of the CAD surface it lies on.
    static Export::VtkGrid fromSurfaceMesh(const SurfaceMesh3D& surfaceMesh);

    /// The restricted facets, sorted by node IDs, with the refiner's view of
    /// each: SurfaceID (index in the topology's surface order), the
    /// BadnessCriterion that failed first (-1 when none) with its
    /// BadnessValue, and Dropped (1 when refinement gave up on the facet).
    static Export::VtkGrid fromRestrictedFacets(
        const MeshData3D& meshData,
        const Topology3D::Topology3D& topology,
        const std::unordered_map<FaceKey, RestrictedFacet, FaceKeyHash>& facets,
        const std::unordered_map<FaceKey, SurfaceFacetCriteria::Badness, FaceKeyHash>& badness,
        const std::unordered_set<FaceKey, FaceKeyHash>& droppedFaces);

    /// The protecting balls: one vertex per weighted node, with Radius and
    /// Weight (the squared radius) as point fields. Glyph with a sphere scaled
    /// by Radius to see them.
    static Export::VtkGrid fromProtectingBalls(const MeshData3D& meshData);
};

} // namespace Meshing
