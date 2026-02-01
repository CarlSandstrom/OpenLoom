#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include <array>
#include <optional>
#include <tuple>

namespace Meshing
{

/// Helper that provides geometric computations for 2D mesh elements requiring node coordinates.
class ElementGeometry2D
{
public:
    explicit ElementGeometry2D(const MeshData2D& mesh);

    /// Computes the circumcircle of a triangle element.
    /// Returns nullopt if the triangle is degenerate.
    std::optional<Circle2D> computeCircumcircle(const TriangleElement& element) const;

    /// Computes the circumcenter of a triangle element.
    /// Returns nullopt if the triangle is degenerate.
    std::optional<Point2D> computeCircumcenter(const TriangleElement& element) const;

    /// Computes the area of a triangle element.
    double computeArea(const TriangleElement& element) const;

    /// Computes the three angles of a triangle element (in radians).
    std::array<double, 3> computeTriangleAngles(const TriangleElement& element) const;

    /// Computes the minimum angle of a triangle element (in radians).
    double computeMinAngle(const TriangleElement& element) const;

    /// Computes the centroid of a triangle element.
    Point2D computeCentroid(const TriangleElement& element) const;

    /// Gets the coordinates of the three nodes of a triangle element.
    std::tuple<Point2D, Point2D, Point2D> getElementNodeCoordinates(const TriangleElement& element) const;

private:
    const MeshData2D& mesh_;
};

} // namespace Meshing
