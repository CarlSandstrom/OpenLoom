#include "Meshing/Core/2D/General/GeometryUtilities2D.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Meshing
{

namespace
{
bool isWithinSegmentBounds(const Point2D& segmentStart, const Point2D& point, const Point2D& segmentEnd,
                           double tolerance);
bool isCollinearInteriorPoint(int orientationSign, const Point2D& segmentStart, const Point2D& point,
                              const Point2D& segmentEnd, double tolerance);
} // namespace

double GeometryUtilities2D::computeEdgeLength(const Point2D& point0, const Point2D& point1)
{
    double deltaX = point1.x() - point0.x();
    double deltaY = point1.y() - point0.y();
    return std::sqrt(deltaX * deltaX + deltaY * deltaY);
}

Circle2D GeometryUtilities2D::createDiametralCircle(const Point2D& point0, const Point2D& point1)
{
    Circle2D circle;
    circle.center = Point2D((point0.x() + point1.x()) * 0.5, (point0.y() + point1.y()) * 0.5);
    circle.radius = computeEdgeLength(point0, point1) * 0.5;
    return circle;
}

bool GeometryUtilities2D::isPointInsideCircle(const Circle2D& circle, const Point2D& point)
{
    double deltaX = point.x() - circle.center.x();
    double deltaY = point.y() - circle.center.y();
    double distanceSquared = deltaX * deltaX + deltaY * deltaY;
    return distanceSquared < circle.radius * circle.radius - 1e-10;
}

std::array<Point2D, 3> GeometryUtilities2D::createSuperTriangle(const std::vector<Point2D>& points)
{
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();

    for (const auto& coordinate : points)
    {
        minX = std::min(minX, coordinate.x());
        minY = std::min(minY, coordinate.y());
        maxX = std::max(maxX, coordinate.x());
        maxY = std::max(maxY, coordinate.y());
    }

    double extentX = maxX - minX;
    double extentY = maxY - minY;
    double maxExtent = std::max(extentX, extentY);
    double centerX = (minX + maxX) * 0.5;
    double centerY = (minY + maxY) * 0.5;

    // A large margin keeps every point well inside the super triangle and away from its edges.
    double scale = 100.0 * maxExtent;

    Point2D vertex0(centerX - scale, centerY - scale);
    Point2D vertex1(centerX + scale, centerY - scale);
    Point2D vertex2(centerX, centerY + scale);
    return {vertex0, vertex1, vertex2};
}

double GeometryUtilities2D::computeSignedArea(const Point2D& point0, const Point2D& point1, const Point2D& point2)
{
    return 0.5 * computeOrientation(point0, point1, point2);
}

double GeometryUtilities2D::computeOrientation(const Point2D& point0, const Point2D& point1, const Point2D& point2)
{
    return (point1.x() - point0.x()) * (point2.y() - point0.y()) - (point1.y() - point0.y()) * (point2.x() - point0.x());
}

int GeometryUtilities2D::computeOrientationSign(const Point2D& point0, const Point2D& point1, const Point2D& point2,
                                                double tolerance)
{
    double value = computeOrientation(point0, point1, point2);

    if (std::abs(value) < tolerance) return 0;
    // Consistent with computeOrientation: positive = counter-clockwise (2), negative = clockwise (1)
    return (value > 0) ? 2 : 1;
}

bool GeometryUtilities2D::segmentsIntersect(const Point2D& segment0Start, const Point2D& segment0End,
                                            const Point2D& segment1Start, const Point2D& segment1End)
{
    int orientation0 = computeOrientationSign(segment0Start, segment0End, segment1Start);
    int orientation1 = computeOrientationSign(segment0Start, segment0End, segment1End);
    int orientation2 = computeOrientationSign(segment1Start, segment1End, segment0Start);
    int orientation3 = computeOrientationSign(segment1Start, segment1End, segment0End);

    if (orientation0 != orientation1 && orientation2 != orientation3) return true;

    // Collinear cases
    if (orientation0 == 0 && isWithinSegmentBounds(segment0Start, segment1Start, segment0End, 0.0)) return true;
    if (orientation1 == 0 && isWithinSegmentBounds(segment0Start, segment1End, segment0End, 0.0)) return true;
    if (orientation2 == 0 && isWithinSegmentBounds(segment1Start, segment0Start, segment1End, 0.0)) return true;
    if (orientation3 == 0 && isWithinSegmentBounds(segment1Start, segment0End, segment1End, 0.0)) return true;

    return false;
}

bool GeometryUtilities2D::segmentsIntersectExcludingSharedEndpoints(const Point2D& segment0Start,
                                                                    const Point2D& segment0End,
                                                                    const Point2D& segment1Start,
                                                                    const Point2D& segment1End, double tolerance)
{
    bool shareEndpoint =
        (segment0Start - segment1Start).norm() < tolerance || (segment0Start - segment1End).norm() < tolerance ||
        (segment0End - segment1Start).norm() < tolerance || (segment0End - segment1End).norm() < tolerance;

    if (!shareEndpoint) return segmentsIntersect(segment0Start, segment0End, segment1Start, segment1End);

    // With a shared endpoint, only a collinear overlap of the interiors counts as an intersection.
    int orientation0 = computeOrientationSign(segment0Start, segment0End, segment1Start);
    int orientation1 = computeOrientationSign(segment0Start, segment0End, segment1End);
    int orientation2 = computeOrientationSign(segment1Start, segment1End, segment0Start);
    int orientation3 = computeOrientationSign(segment1Start, segment1End, segment0End);

    return isCollinearInteriorPoint(orientation0, segment0Start, segment1Start, segment0End, tolerance) ||
           isCollinearInteriorPoint(orientation1, segment0Start, segment1End, segment0End, tolerance) ||
           isCollinearInteriorPoint(orientation2, segment1Start, segment0Start, segment1End, tolerance) ||
           isCollinearInteriorPoint(orientation3, segment1Start, segment0End, segment1End, tolerance);
}

bool GeometryUtilities2D::isPointStrictlyInsideTriangle(const Point2D& point, const Point2D& vertex0,
                                                        const Point2D& vertex1, const Point2D& vertex2,
                                                        double tolerance)
{
    // Barycentric coordinates method
    double edge0X = vertex1.x() - vertex0.x();
    double edge0Y = vertex1.y() - vertex0.y();
    double edge1X = vertex2.x() - vertex0.x();
    double edge1Y = vertex2.y() - vertex0.y();
    double offsetX = point.x() - vertex0.x();
    double offsetY = point.y() - vertex0.y();

    double dot00 = edge0X * edge0X + edge0Y * edge0Y;
    double dot01 = edge0X * edge1X + edge0Y * edge1Y;
    double dot02 = edge0X * offsetX + edge0Y * offsetY;
    double dot11 = edge1X * edge1X + edge1Y * edge1Y;
    double dot12 = edge1X * offsetX + edge1Y * offsetY;

    double inverseDenominator = 1.0 / (dot00 * dot11 - dot01 * dot01);
    double u = (dot11 * dot02 - dot01 * dot12) * inverseDenominator;
    double v = (dot00 * dot12 - dot01 * dot02) * inverseDenominator;

    return (u > tolerance) && (v > tolerance) && (u + v < 1.0 - tolerance);
}

bool GeometryUtilities2D::isPointInsideOrOnTriangle(const Point2D& point, const Point2D& vertex0,
                                                    const Point2D& vertex1, const Point2D& vertex2)
{
    double orientation0 = computeOrientation(vertex0, vertex1, point);
    double orientation1 = computeOrientation(vertex1, vertex2, point);
    double orientation2 = computeOrientation(vertex2, vertex0, point);
    bool hasNegative = (orientation0 < 0) || (orientation1 < 0) || (orientation2 < 0);
    bool hasPositive = (orientation0 > 0) || (orientation1 > 0) || (orientation2 > 0);
    return !(hasNegative && hasPositive);
}

namespace
{
/// Tests whether point lies in the bounding box of the segment, widened by tolerance.
/// Callers have already established that the three points are collinear.
bool isWithinSegmentBounds(const Point2D& segmentStart, const Point2D& point, const Point2D& segmentEnd,
                           double tolerance)
{
    return point.x() <= std::max(segmentStart.x(), segmentEnd.x()) + tolerance &&
           point.x() >= std::min(segmentStart.x(), segmentEnd.x()) - tolerance &&
           point.y() <= std::max(segmentStart.y(), segmentEnd.y()) + tolerance &&
           point.y() >= std::min(segmentStart.y(), segmentEnd.y()) - tolerance;
}

/// Tests whether point is collinear with the segment and lies within it, away from both endpoints.
bool isCollinearInteriorPoint(int orientationSign, const Point2D& segmentStart, const Point2D& point,
                              const Point2D& segmentEnd, double tolerance)
{
    return orientationSign == 0 && isWithinSegmentBounds(segmentStart, point, segmentEnd, tolerance) &&
           (point - segmentStart).norm() > tolerance && (point - segmentEnd).norm() > tolerance;
}
} // namespace

} // namespace Meshing
