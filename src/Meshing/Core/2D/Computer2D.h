#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include <optional>
#include <unordered_map>

namespace Meshing
{

/// Helper that owns 2D mesh data and exposes computations that require node coordinates.
class Computer2D
{
public:
    explicit Computer2D(const MeshData2D& mesh);

    // Methods that use mesh data
    std::optional<CircumCircle2D> computeCircumcircle(const TriangleElement& tri) const;
    double computeArea(const TriangleElement& element) const;
    std::array<Point2D, 3> createSuperTriangle(const std::vector<Point2D>& points);

    static bool isPointInsideCircumcircle(const CircumCircle2D& circle, const Point2D& point);

private:
    std::tuple<Point2D, Point2D, Point2D> getElementNodeCoordinates(const TriangleElement& element) const;

    const MeshData2D& mesh_;
};

} // namespace Meshing
