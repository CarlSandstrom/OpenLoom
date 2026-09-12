#pragma once

#include "Common/Types.h"
#include "Meshing/Data/CurveSegmentManager.h"

#include <cstddef>
#include <map>

namespace Geometry3D
{
class GeometryCollection3D;
}

namespace Topology3D
{
class Topology3D;
}

namespace Meshing
{

struct DiscretizationResult3D;

/// Builds the curve segments that record where the model's edge curves run
/// through the mesh. Runs once, from RCDTMesher::buildInitial(); the segments it
/// produces are then split in place by refinement (see CurveSegmentGeometry).
class CurveSegmentBuilder
{
public:
    /// One segment per consecutive node pair along each topology edge, skipping
    /// seam twin and degenerate edges.
    /// discretization supplies the boundary samples: its edgeIdToPointIndicesMap gives the
    /// ordered point-index sequence per edge (including endpoints), and its edgeParameters
    /// gives the edge parameter of each of those points. Taking the whole result rather than
    /// the two fields separately is what keeps those point indices referring to the same
    /// sampling.
    /// pointIndexToNodeIdMap converts those point indices to the mesh node IDs the
    /// triangulation gave them.
    static CurveSegmentManager build(const Topology3D::Topology3D& topology,
                                     const Geometry3D::GeometryCollection3D& geometry,
                                     const DiscretizationResult3D& discretization,
                                     const std::map<size_t, size_t>& pointIndexToNodeIdMap);
};

} // namespace Meshing
