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
    static Circle2D createDiametralCircle(const Point2D& p1, const Point2D& p2);

    /// Tests if a point is inside the circle.
    static bool isPointInsideCircle(const Circle2D& circle, const Point2D& point);

    /// Creates a super triangle that contains all given points.
    /// Used for Delaunay triangulation initialization.
    static std::array<Point2D, 3> createSuperTriangle(const std::vector<Point2D>& points);

    /// Computes the signed area of a triangle.
    /// Returns positive for counter-clockwise orientation, negative for clockwise.
    static double computeSignedArea(const Point2D& p1, const Point2D& p2, const Point2D& p3);

    /// Computes the orientation value (cross product) of three points.
    /// Returns: positive for CCW, negative for CW, zero for collinear.
    static double computeOrientation(const Point2D& p, const Point2D& q, const Point2D& r);

    /// Computes the orientation sign of three points with tolerance.
    /// Returns: 0 for collinear, 1 for clockwise, 2 for counter-clockwise.
    static int computeOrientationSign(const Point2D& p, const Point2D& q, const Point2D& r,
                                      double tolerance = 1e-10);

    /// Checks if two line segments intersect.
    /// Returns true if segments properly intersect (including collinear overlap cases).
    static bool segmentsIntersect(const Point2D& a1, const Point2D& a2,
                                  const Point2D& b1, const Point2D& b2);

    /// Checks if two segments intersect in their interiors, excluding shared endpoints.
    /// Adjacent triangle edges that share a vertex are not considered intersecting.
    static bool segmentsIntersectExcludingSharedEndpoints(const Point2D& a1, const Point2D& a2,
                                                          const Point2D& b1, const Point2D& b2,
                                                          double tolerance = 1e-10);

    /// Tests if a point is strictly inside a triangle (excluding boundary).
    /// Uses barycentric coordinates with tolerance.
    static bool isPointStrictlyInsideTriangle(const Point2D& point,
                                              const Point2D& v0, const Point2D& v1, const Point2D& v2,
                                              double tolerance = 1e-10);

    /// Tests if a point is inside or on a triangle (including boundary).
    /// Uses orientation-based test.
    static bool isPointInsideOrOnTriangle(const Point2D& point,
                                          const Point2D& v0, const Point2D& v1, const Point2D& v2);
};

} // namespace Meshing
