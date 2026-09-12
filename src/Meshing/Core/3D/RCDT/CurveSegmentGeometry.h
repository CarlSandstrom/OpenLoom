#pragma once

#include "Common/Types.h"
#include "Meshing/Data/CurveSegmentManager.h"

namespace Geometry3D
{
class GeometryCollection3D;
}

namespace Meshing
{

/// Geometric queries about a CurveSegment that have to consult the CAD curve it
/// lies on. A CurveSegment itself only records the node pair and the parameter
/// range, so anything about where it actually runs in space is answered here.
class CurveSegmentGeometry
{
public:
    /// Returns the 3D point on the edge curve at the arc-length midpoint of the segment.
    static Point3D splitPoint(const CurveSegment& segment,
                              const Geometry3D::GeometryCollection3D& geometry);
};

} // namespace Meshing
