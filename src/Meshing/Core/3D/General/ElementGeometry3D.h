#pragma once

#include "Common/Types.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include <optional>

namespace Meshing
{

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
    /// when every weight is 0. A tetrahedron flat to rounding gets the
    /// equal-power point in its plane; nullopt only when three of its vertices
    /// are collinear.
    std::optional<Point3D> computeOrthocenter(const TetrahedralElement& element) const;

    /// The weighted circumcenter of a triangle: the point in its plane with
    /// equal power distance to all three weighted vertices. It lies on the
    /// line carrying the triangle's dual power-diagram edge. Returns nullopt
    /// if the triangle is degenerate.
    std::optional<Point3D> computeOrthocenter(const TriangleElement& element) const;

private:
    const MeshData3D& mesh_;
};

} // namespace Meshing
