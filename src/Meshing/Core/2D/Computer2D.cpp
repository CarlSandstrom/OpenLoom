#include "Computer2D.h"

#include <cmath>
#include <optional>

namespace Meshing
{

Computer2D::Computer2D(const MeshData2D& mesh) :
    mesh_(mesh)
{
}

std::optional<CircumCircle2D> Computer2D::computeCircumcircle(const TriangleElement& tri) const
{
    auto [p0, p1, p2] = getElementNodeCoordinates(tri);

    const double ax = p1.x() - p0.x();
    const double ay = p1.y() - p0.y();
    const double bx = p2.x() - p0.x();
    const double by = p2.y() - p0.y();

    const double d = 2.0 * (ax * by - ay * bx);

    if (std::abs(d) < 1e-10)
    {
        // Degenerate triangle
        return std::nullopt;
    }

    const double aSq = ax * ax + ay * ay;
    const double bSq = bx * bx + by * by;

    const double cx = (by * aSq - ay * bSq) / d;
    const double cy = (ax * bSq - bx * aSq) / d;

    CircumCircle2D circle;
    circle.center = Point2D(p0.x() + cx, p0.y() + cy);
    circle.radius = std::sqrt(cx * cx + cy * cy);

    return circle;
}

double Computer2D::computeArea(const TriangleElement& element) const
{
    auto [v0, v1, v2] = getElementNodeCoordinates(element);
    const double dx1 = v1.x() - v0.x();
    const double dy1 = v1.y() - v0.y();
    const double dx2 = v2.x() - v0.x();
    const double dy2 = v2.y() - v0.y();

    // Cross product in 2D gives the area
    return 0.5 * std::abs(dx1 * dy2 - dy1 * dx2);
}

std::array<Point2D, 3> Computer2D::createSuperTriangle(const std::vector<Point2D>& points)
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
    const double scale = 10.0 * dmax;

    Point2D p0(midX - scale, midY - scale);
    Point2D p1(midX + scale, midY - scale);
    Point2D p2(midX, midY + scale);
    return {p0, p1, p2};
}

std::tuple<Point2D, Point2D, Point2D> Computer2D::getElementNodeCoordinates(const TriangleElement& element) const
{
    auto nodeIds = element.getNodeIds();
    const auto* n0 = mesh_.getNode(nodeIds[0]);
    const auto* n1 = mesh_.getNode(nodeIds[1]);
    const auto* n2 = mesh_.getNode(nodeIds[2]);
    return {n0->getCoordinates(), n1->getCoordinates(), n2->getCoordinates()};
}

bool Computer2D::isPointInsideCircumcircle(const CircumCircle2D& circle, const Point2D& point)
{
    const double dx = point.x() - circle.center.x();
    const double dy = point.y() - circle.center.y();
    const double distSquared = dx * dx + dy * dy;

    return distSquared < circle.radius * circle.radius - 1e-10;
}

} // namespace Meshing
