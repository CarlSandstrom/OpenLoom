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

DiametralCircle2D GeometryUtilities2D::createDiametralCircle(const Point2D& p1, const Point2D& p2)
{
    DiametralCircle2D circle;
    circle.center = Point2D((p1.x() + p2.x()) * 0.5, (p1.y() + p2.y()) * 0.5);
    circle.radius = computeEdgeLength(p1, p2) * 0.5;
    return circle;
}

bool GeometryUtilities2D::isPointInDiametralCircle(const DiametralCircle2D& circle, const Point2D& point)
{
    const double dx = point.x() - circle.center.x();
    const double dy = point.y() - circle.center.y();
    const double distSquared = dx * dx + dy * dy;
    return distSquared < circle.radius * circle.radius - 1e-10;
}

bool GeometryUtilities2D::isPointInsideCircumcircle(const CircumCircle2D& circle, const Point2D& point)
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

} // namespace Meshing
