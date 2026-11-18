#pragma once

#include "Common/Types.h"
#include <array>
#include <optional>

namespace Meshing
{

class MeshData3D;
class TetrahedronElement;
class TriangleElement;

/// Helper that provides geometric computations for 3D mesh elements requiring node coordinates.
class ElementGeometry3D
{
public:
    explicit ElementGeometry3D(const MeshData3D& mesh);

    /// The weighted circumcenter (orthocenter) of a tetrahedral element: the
    /// point with equal power distance |x - p|^2 - w to all four weighted
    /// vertices. In a regular triangulation this, not the circumcenter, is the
    /// element's vertex of the dual power diagram. Equals the circumcenter
    /// when every weight is 0. Computed in floating point when a rounding
    /// error bound certifies the result, and with exact determinants
    /// otherwise, so even a sliver's orthocenter is accurate and on the
    /// correct side of any surface its vertices lie on (OPE-214). An exactly
    /// flat
    /// tetrahedron gets the equal-power point in its plane; nullopt only when
    /// three of its vertices are collinear.
    std::optional<Point3D> computeOrthocenter(const TetrahedronElement& element) const;

    /// The weighted circumcenter of a triangle: the point in its plane with
    /// equal power distance to all three weighted vertices. It lies on the
    /// line carrying the triangle's dual power-diagram edge. Returns nullopt
    /// if the triangle is degenerate.
    std::optional<Point3D> computeOrthocenter(const TriangleElement& element) const;

    /// The smallest of a tetrahedron's six dihedral angles, in degrees: 0
    /// for a flat one. Slivers have well-shaped faces and a radius-edge
    /// ratio near the ideal, so this is the measure that finds them.
    static double computeMinimumDihedralAngle(const std::array<Point3D, 4>& points);

private:
    const MeshData3D& mesh_;
};

} // namespace Meshing
