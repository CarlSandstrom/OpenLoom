#include "GeometryUtilities2D.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace Meshing
{

double GeometryUtilities2D::computeEdgeLength(const Point2D& p1, const Point2D& p2)
{
    const double dx = p2.x() - p1.x();
    const double dy = p2.y() - p1.y();
    return std::sqrt(dx * dx + dy * dy);
}

Circle2D GeometryUtilities2D::createDiametralCircle(const Point2D& p1, const Point2D& p2)
{
    Circle2D circle;
    circle.center = Point2D((p1.x() + p2.x()) * 0.5, (p1.y() + p2.y()) * 0.5);
    circle.radius = computeEdgeLength(p1, p2) * 0.5;
    return circle;
}

bool GeometryUtilities2D::isPointInsideCircle(const Circle2D& circle, const Point2D& point)
{
    const double dx = point.x() - circle.center.x();
    const double dy = point.y() - circle.center.y();
    const double distSquared = dx * dx + dy * dy;
    return distSquared < circle.radius * circle.radius - 1e-10;
}

std::array<Point2D, 3> GeometryUtilities2D::createSuperTriangle(const std::vector<Point2D>& points)
{
    // Find bounding box
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();

    for (const auto& coord : points)
    {
        minX = std::min(minX, coord.x());
        minY = std::min(minY, coord.y());
        maxX = std::max(maxX, coord.x());
        maxY = std::max(maxY, coord.y());
    }

    const double dx = maxX - minX;
    const double dy = maxY - minY;
    const double dmax = std::max(dx, dy);
    const double midX = (minX + maxX) * 0.5;
    const double midY = (minY + maxY) * 0.5;

    // Create node points describing a large triangle that contains all points
    // Use a large scale factor to ensure all points are well within the super triangle
    // and to avoid numerical precision issues
    const double scale = 100.0 * dmax;

    Point2D p0(midX - scale, midY - scale);
    Point2D p1(midX + scale, midY - scale);
    Point2D p2(midX, midY + scale);
    return {p0, p1, p2};
}

double GeometryUtilities2D::computeSignedArea(const Point2D& p1, const Point2D& p2, const Point2D& p3)
{
    return 0.5 * ((p2.x() - p1.x()) * (p3.y() - p1.y()) -
                  (p3.x() - p1.x()) * (p2.y() - p1.y()));
}

double GeometryUtilities2D::computeOrientation(const Point2D& p, const Point2D& q, const Point2D& r)
{
    return (q.x() - p.x()) * (r.y() - p.y()) - (q.y() - p.y()) * (r.x() - p.x());
}

int GeometryUtilities2D::computeOrientationSign(const Point2D& p, const Point2D& q, const Point2D& r,
                                                double tolerance)
{
    double val = (q.y() - p.y()) * (r.x() - q.x()) -
                 (q.x() - p.x()) * (r.y() - q.y());

    if (std::abs(val) < tolerance) return 0;
    return (val > 0) ? 1 : 2;
}

bool GeometryUtilities2D::segmentsIntersect(const Point2D& a1, const Point2D& a2,
                                            const Point2D& b1, const Point2D& b2)
{
    auto onSegment = [](const Point2D& p, const Point2D& q, const Point2D& r) -> bool
    {
        return q.x() <= std::max(p.x(), r.x()) && q.x() >= std::min(p.x(), r.x()) &&
               q.y() <= std::max(p.y(), r.y()) && q.y() >= std::min(p.y(), r.y());
    };

    int o1 = computeOrientationSign(a1, a2, b1);
    int o2 = computeOrientationSign(a1, a2, b2);
    int o3 = computeOrientationSign(b1, b2, a1);
    int o4 = computeOrientationSign(b1, b2, a2);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special cases for collinear points
    if (o1 == 0 && onSegment(a1, b1, a2)) return true;
    if (o2 == 0 && onSegment(a1, b2, a2)) return true;
    if (o3 == 0 && onSegment(b1, a1, b2)) return true;
    if (o4 == 0 && onSegment(b1, a2, b2)) return true;

    return false;
}

} // namespace Meshing
