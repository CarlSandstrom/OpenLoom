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

class PeriodicMeshData2D;

/// Helper that provides geometric computations for 2D mesh elements requiring node coordinates.
///
/// When a PeriodicMeshData2D pointer is provided at construction, the ID-based overloads
/// use offset-aware coordinates (vertices shifted by their periodic offset) instead of
/// the canonical coordinates stored in MeshData2D.  The TriangleElement& overloads
/// always use canonical coordinates and are unaffected.
class ElementGeometry2D
{
public:
    /// Constructs a non-periodic geometry helper.
    explicit ElementGeometry2D(const MeshData2D& mesh);

    /// Constructs a periodic-aware geometry helper.
    /// periodicData may be null (equivalent to the non-periodic constructor).
    ElementGeometry2D(const MeshData2D& mesh, PeriodicMeshData2D* periodicData);

    // --- TriangleElement& overloads (use canonical coordinates) ---

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

    /// Gets the coordinates of the three nodes of a triangle element (canonical).
    std::tuple<Point2D, Point2D, Point2D> getElementNodeCoordinates(const TriangleElement& element) const;

    // --- Element ID overloads (use periodic offset-aware coordinates when available) ---

    /// Computes the circumcircle using offset-aware coordinates.
    std::optional<Circle2D> computeCircumcircle(size_t elementId) const;

    /// Computes the circumcenter using offset-aware coordinates.
    std::optional<Point2D> computeCircumcenter(size_t elementId) const;

    /// Computes the centroid using offset-aware coordinates.
    Point2D computeCentroid(size_t elementId) const;

    /// Gets the offset-aware coordinates of a triangle's three nodes.
    std::tuple<Point2D, Point2D, Point2D> getElementNodeCoordinates(size_t elementId) const;

private:
    const MeshData2D& mesh_;
    PeriodicMeshData2D* periodicData_ = nullptr;
};

} // namespace Meshing
