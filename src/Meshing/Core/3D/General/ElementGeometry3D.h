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

    /// Computes the circumscribing sphere of a tetrahedral element.
    /// Returns nullopt if the tetrahedron is degenerate.
    std::optional<CircumscribedSphere> computeCircumscribingSphere(const TetrahedralElement& element) const;

    /// Tests if a point is inside the circumscribing sphere of a tetrahedral element.
    bool isPointInsideCircumscribingSphere(const TetrahedralElement& element,
                                           const Point3D& point,
                                           double tolerance = 1e-12) const;

    /// Computes the centroid (center of mass) of a tetrahedral element.
    Point3D computeCentroid(const TetrahedralElement& element) const;

private:
    std::tuple<Point3D, Point3D, Point3D, Point3D> getElementNodeCoordinates(const TetrahedralElement& element) const;
    std::tuple<Point3D, Point3D, Point3D> getElementNodeCoordinates(const TriangleElement& element) const;

    const MeshData3D& mesh_;
};

} // namespace Meshing
