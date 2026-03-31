#pragma once

#include "Common/Types.h"
#include "Meshing/Data/CurveSegmentManager.h"

#include <string>
#include <unordered_map>

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
/// cornerIdToNodeId maps each topology corner ID to the mesh node ID at that corner.
void buildCurveSegments(CurveSegmentManager& manager,
                        const Topology3D::Topology3D& topology,
                        const Geometry3D::GeometryCollection3D& geometry,
                        const std::unordered_map<std::string, size_t>& cornerIdToNodeId);

/// Returns the 3D point on the edge curve at the arc-length midpoint of the segment.
Point3D computeSplitPoint(const CurveSegment& segment,
                          const Geometry3D::GeometryCollection3D& geometry);

} // namespace Meshing
