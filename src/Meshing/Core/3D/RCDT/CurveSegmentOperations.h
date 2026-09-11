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

/// Builds the curve segments that record where the model's edge curves run
/// through the mesh, and computes where to split one.
class CurveSegmentOperations
{
public:
    /// One segment per consecutive node pair along each topology edge, skipping
    /// seam twin and degenerate edges.
    /// edgeIdToPointIndicesMap gives the ordered point-index sequence per edge (including endpoints).
    /// pointIndexToNodeIdMap converts point indices to mesh node IDs.
    /// edgeParameters stores per-point edge parameter values; interior points have one entry each.
    static CurveSegmentManager buildCurveSegments(const Topology3D::Topology3D& topology,
                                                  const Geometry3D::GeometryCollection3D& geometry,
                                                  const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap,
                                                  const std::map<size_t, size_t>& pointIndexToNodeIdMap,
                                                  const std::vector<std::vector<double>>& edgeParameters);

    /// Returns the 3D point on the edge curve at the arc-length midpoint of the segment.
    static Point3D computeSplitPoint(const CurveSegment& segment,
                                     const Geometry3D::GeometryCollection3D& geometry);
};

} // namespace Meshing
