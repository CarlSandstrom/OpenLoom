#pragma once

#include "Common/Types.h"
#include "Meshing/Data/MeshData2D.h"
#include "Meshing/Data/TriangleElement.h"
#include <optional>
#include <unordered_map>

namespace Meshing
{

/// Helper that owns 2D mesh data and exposes computations that require node coordinates.
class Computer2D
{
public:
    struct CircumCircle2D
    {
        Point2D center;
        double radiusSquared;
    };

    explicit Computer2D(const MeshData2D& mesh);

    // Methods that use mesh data
    std::optional<CircumCircle2D> computeCircumcircle(const TriangleElement& tri) const;
    double computeArea(const TriangleElement& element) const;

    // Static methods for working with external coordinate maps (used during triangulation)
    static std::optional<CircumCircle2D> computeCircumcircle(const TriangleElement& tri,
                                                             const std::unordered_map<size_t, Point2D>& nodeCoords);

    static bool isPointInsideCircumcircle(const CircumCircle2D& circle, const Point2D& point);

private:
    std::tuple<Point2D, Point2D, Point2D> getElementNodeCoordinates(const TriangleElement& element) const;

    const MeshData2D& mesh_;
};

} // namespace Meshing
