#pragma once

#include "Common/Types.h"
#include "Meshing/Data/CurveSegmentManager.h"

#include <map>
#include <string>
#include <vector>

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

/// Populates manager from topology edges, skipping seam twin edges.
/// edgeIdToPointIndicesMap gives the ordered point-index sequence per edge (including endpoints).
/// pointIndexToNodeIdMap converts point indices to mesh node IDs.
/// edgeParameters stores per-point edge parameter values; interior points have one entry each.
void buildCurveSegments(CurveSegmentManager& manager,
                        const Topology3D::Topology3D& topology,
                        const Geometry3D::GeometryCollection3D& geometry,
                        const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap,
                        const std::map<size_t, size_t>& pointIndexToNodeIdMap,
                        const std::vector<std::vector<double>>& edgeParameters);

/// Returns the 3D point on the edge curve at the arc-length midpoint of the segment.
Point3D computeSplitPoint(const CurveSegment& segment,
                          const Geometry3D::GeometryCollection3D& geometry);

} // namespace Meshing
