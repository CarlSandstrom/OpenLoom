#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/General/GeometryStructures2D.h"
#include <array>
#include <vector>

namespace Meshing
{

/// Static utilities for pure 2D geometric computations that don't require mesh context.
class GeometryUtilities2D
{
public:
    /// Computes the Euclidean distance between two points.
    static double computeEdgeLength(const Point2D& point0, const Point2D& point1);

    /// Creates a diametral circle with the given segment as diameter.
    static Circle2D createDiametralCircle(const Point2D& point0, const Point2D& point1);

    /// Tests if a point is inside the circle.
    static bool isPointInsideCircle(const Circle2D& circle, const Point2D& point);

    /// Creates a super triangle that contains all given points.
    /// Used for Delaunay triangulation initialization.
    static std::array<Point2D, 3> createSuperTriangle(const std::vector<Point2D>& points);

    /// Computes the signed area of a triangle.
    /// Returns positive for counter-clockwise orientation, negative for clockwise.
    static double computeSignedArea(const Point2D& point0, const Point2D& point1, const Point2D& point2);

    /// Computes the orientation value (cross product) of three points.
    /// Returns: positive for counter-clockwise, negative for clockwise, zero for collinear.
    static double computeOrientation(const Point2D& point0, const Point2D& point1, const Point2D& point2);

    /// Computes the orientation sign of three points with tolerance.
    /// Returns: 0 for collinear, 1 for clockwise, 2 for counter-clockwise.
    static int computeOrientationSign(const Point2D& point0, const Point2D& point1, const Point2D& point2,
                                      double tolerance = 1e-10);

    /// Checks if two line segments intersect.
    /// Returns true if segments properly intersect (including collinear overlap cases).
    static bool segmentsIntersect(const Point2D& segment0Start, const Point2D& segment0End,
                                  const Point2D& segment1Start, const Point2D& segment1End);

    /// Checks if two segments intersect in their interiors, excluding shared endpoints.
    /// Adjacent triangle edges that share a vertex are not considered intersecting.
    static bool segmentsIntersectExcludingSharedEndpoints(const Point2D& segment0Start, const Point2D& segment0End,
                                                          const Point2D& segment1Start, const Point2D& segment1End,
                                                          double tolerance = 1e-10);

    /// Tests if a point is strictly inside a triangle (excluding boundary).
    /// Uses barycentric coordinates with tolerance.
    static bool isPointStrictlyInsideTriangle(const Point2D& point,
                                              const Point2D& vertex0, const Point2D& vertex1, const Point2D& vertex2,
                                              double tolerance = 1e-10);

    /// Tests if a point is inside or on a triangle (including boundary).
    /// Uses orientation-based test.
    static bool isPointInsideOrOnTriangle(const Point2D& point,
                                          const Point2D& vertex0, const Point2D& vertex1, const Point2D& vertex2);
};

} // namespace Meshing
