#pragma once

#include "Common/Types.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include <optional>
#include <tuple>

namespace Meshing
{

class TriangleElement;

/// Helper that provides geometric computations for 3D mesh elements requiring node coordinates.
class ElementGeometry3D
{
public:
    explicit ElementGeometry3D(const MeshData3D& mesh);

    /// Computes the volume of a tetrahedral element.
    double computeVolume(const TetrahedralElement& element) const;

    /// Computes the area of a triangular face in 3D.
    double computeArea(const TriangleElement& element) const;

    /// Computes the outward unit normal of a triangular face in 3D.
    /// Returns the zero vector if the triangle is degenerate.
    Point3D computeNormal(const TriangleElement& element) const;

    /// Computes the circumscribed circle of a triangular face in 3D.
    /// The circle center lies in the plane of the triangle.
    /// Returns nullopt if the triangle is degenerate (collinear vertices).
    std::optional<EquatorialSphere> computeCircumcircle(const TriangleElement& element) const;

    /// Computes the circumscribing sphere of a tetrahedral element.
    /// Returns nullopt if the tetrahedron is degenerate.
    std::optional<CircumscribedSphere> computeCircumscribingSphere(const TetrahedralElement& element) const;

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

    /// Computes the centroid (center of mass) of a tetrahedral element.
    Point3D computeCentroid(const TetrahedralElement& element) const;

private:
    std::tuple<Point3D, Point3D, Point3D, Point3D> getElementNodeCoordinates(const TetrahedralElement& element) const;
    std::tuple<Point3D, Point3D, Point3D> getElementNodeCoordinates(const TriangleElement& element) const;

    const MeshData3D& mesh_;
};

} // namespace Meshing
