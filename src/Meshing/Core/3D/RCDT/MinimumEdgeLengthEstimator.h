#pragma once

#include "Common/Types.h"

#include <vector>

namespace Meshing
{

class SizingField3D;

/// Derives RCDT's minimum edge length when the caller does not set one: what
/// WeightedDualRestriction scales its tessellations' cell size by.
///
/// The floor is a sliver guard, not a size target: refinement must be free to
/// reach the size being asked for, so the floor has to sit well below it.
class MinimumEdgeLengthEstimator
{
public:
    /// The median nearest-neighbor distance among points, divided by 10. Median
    /// rather than minimum because a periodic curve's discretization can leave a
    /// short remainder segment near its seam vertex that is not representative.
    static double fromPointSpacing(const std::vector<Point3D>& points);

    /// The floor when a sizing field is available (OPE-181). A low percentile of
    /// the field's source sizes, divided by 10.
    ///
    /// fromPointSpacing() would be circular here: a discretization finer because
    /// of h(x) lowers the floor, which rebuilds the tessellations finer too. The
    /// sources are set by the geometry and do not move with the discretization.
    static double fromSizingField(const SizingField3D& sizingField);
};

} // namespace Meshing
