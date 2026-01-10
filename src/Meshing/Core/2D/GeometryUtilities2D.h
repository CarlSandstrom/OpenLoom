#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include <array>
#include <vector>

namespace Meshing
{

/// Static utilities for pure 2D geometric computations that don't require mesh context.
class GeometryUtilities2D
{
public:
    /// Computes the Euclidean distance between two points.
    static double computeEdgeLength(const Point2D& p1, const Point2D& p2);

    /// Creates a diametral circle with the given segment as diameter.
    static DiametralCircle2D createDiametralCircle(const Point2D& p1, const Point2D& p2);

    /// Tests if a point is inside the diametral circle.
    static bool isPointInDiametralCircle(const DiametralCircle2D& circle, const Point2D& point);

    /// Tests if a point is inside the circumcircle.
    static bool isPointInsideCircumcircle(const CircumCircle2D& circle, const Point2D& point);

    /// Creates a super triangle that contains all given points.
    /// Used for Delaunay triangulation initialization.
    static std::array<Point2D, 3> createSuperTriangle(const std::vector<Point2D>& points);
};

} // namespace Meshing
