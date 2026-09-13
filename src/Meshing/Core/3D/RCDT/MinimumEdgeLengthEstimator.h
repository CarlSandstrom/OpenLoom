#pragma once

#include "Common/Types.h"

#include <vector>

namespace Meshing
{

class SizingField3D;

/// Derives RCDT's minimum edge length when the caller does not set one: the
/// size floor RCDTRefiner stops refining at, CurveProtectionSubdivider
/// subdivides against, and RestrictedTriangulation scales its tessellation
/// oracle's cell size by.
///
/// The floor is a SLIVER GUARD, not a size target: refinement must be free to
/// reach the size actually being asked for, so the floor has to sit well below
/// it. Setting it AT the target disables refinement wherever the mesh has
/// arrived, which collapses the torus to degenerate triangles.
class MinimumEdgeLengthEstimator
{
public:
    /// The median nearest-neighbor distance among points, divided by 10.
    /// Median rather than minimum because a periodic curve's discretization
    /// can leave a short "remainder" segment near its seam vertex that isn't
    /// representative of the intended spacing (see project memory: Linear
    /// ticket on removing OCC seams).
    static double fromPointSpacing(const std::vector<Point3D>& points);

    /// The floor when an explicit sizing field is available (OPE-181).
    ///
    /// Deriving it from the discretization's own spacing, as fromPointSpacing()
    /// does, is CIRCULAR once that spacing comes from h(x): making the boundary
    /// finer lowers the floor, which lets refinement chase proportionally
    /// deeper, and -- because RestrictedTriangulation scales its tessellation
    /// oracle's cell size by this same value -- rebuilds the oracle finer at the
    /// same time. Measured on SaddleSurfaceMesh: bounding segment length by h(x)
    /// moved the derived floor 0.0523 -> 0.0212 and the non-manifold count
    /// 17 -> 388, of which 349 disappeared again when the floor alone was held
    /// at its old value.
    ///
    /// h's own SOURCES are the non-circular quantity: they are set by the
    /// geometry, so they do not move when the discretization does.
    ///
    /// Which source to read is a separate question from that circularity, and
    /// the answer is the same as fromPointSpacing()'s: a percentile, not the
    /// extreme. Curvature and local-feature-size sampling routinely produces a
    /// few sources asking for elements far smaller than the rest of the model
    /// needs, and the single smallest is as unrepresentative here as a seam
    /// remainder segment is there. Measured on SaddleSurfaceMesh with the field
    /// enabled (OPE-180): the minimum gives a floor of 0.0258 and 9 non-manifold
    /// edges, the median 0.0523 and 2, against 1 for the same model with no
    /// field at all.
    static double fromSizingField(const SizingField3D& sizingField);
};

} // namespace Meshing
