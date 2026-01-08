#include "Computer2D.h"

#include <algorithm>
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
    // Use a large scale factor to ensure all points are well within the super triangle
    // and to avoid numerical precision issues
    const double scale = 100.0 * dmax;

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

double Computer2D::computeEdgeLength(const Point2D& p1, const Point2D& p2)
{
    const double dx = p2.x() - p1.x();
    const double dy = p2.y() - p1.y();
    return std::sqrt(dx * dx + dy * dy);
}

double Computer2D::computeShortestEdgeLength(const TriangleElement& element) const
{
    auto [p0, p1, p2] = getElementNodeCoordinates(element);
    const double e0 = computeEdgeLength(p0, p1);
    const double e1 = computeEdgeLength(p1, p2);
    const double e2 = computeEdgeLength(p2, p0);
    return std::min({e0, e1, e2});
}

double Computer2D::computeLongestEdgeLength(const TriangleElement& element) const
{
    auto [p0, p1, p2] = getElementNodeCoordinates(element);
    const double e0 = computeEdgeLength(p0, p1);
    const double e1 = computeEdgeLength(p1, p2);
    const double e2 = computeEdgeLength(p2, p0);
    return std::max({e0, e1, e2});
}

std::optional<double> Computer2D::computeCircumradiusToShortestEdgeRatio(const TriangleElement& element) const
{
    auto circumcircle = computeCircumcircle(element);
    if (!circumcircle.has_value())
    {
        return std::nullopt;
    }
    const double shortestEdge = computeShortestEdgeLength(element);
    if (shortestEdge < 1e-10)
    {
        return std::nullopt;
    }
    return circumcircle->radius / shortestEdge;
}

std::array<double, 3> Computer2D::computeTriangleAngles(const TriangleElement& element) const
{
    auto [p0, p1, p2] = getElementNodeCoordinates(element);

    const double a = computeEdgeLength(p1, p2);
    const double b = computeEdgeLength(p0, p2);
    const double c = computeEdgeLength(p0, p1);

    const double angle0 = std::acos((b * b + c * c - a * a) / (2.0 * b * c));
    const double angle1 = std::acos((a * a + c * c - b * b) / (2.0 * a * c));
    const double angle2 = std::acos((a * a + b * b - c * c) / (2.0 * a * b));

    return {angle0, angle1, angle2};
}

double Computer2D::computeMinAngle(const TriangleElement& element) const
{
    auto angles = computeTriangleAngles(element);
    return std::min({angles[0], angles[1], angles[2]});
}

DiametralCircle2D Computer2D::createDiametralCircle(const Point2D& p1, const Point2D& p2)
{
    DiametralCircle2D circle;
    circle.center = Point2D((p1.x() + p2.x()) * 0.5, (p1.y() + p2.y()) * 0.5);
    circle.radius = computeEdgeLength(p1, p2) * 0.5;
    return circle;
}

bool Computer2D::isPointInDiametralCircle(const DiametralCircle2D& circle, const Point2D& point)
{
    const double dx = point.x() - circle.center.x();
    const double dy = point.y() - circle.center.y();
    const double distSquared = dx * dx + dy * dy;
    return distSquared < circle.radius * circle.radius - 1e-10;
}

bool Computer2D::isSegmentEncroached(const ConstrainedSegment2D& segment, const Point2D& point) const
{
    const auto* node1 = mesh_.getNode(segment.nodeId1);
    const auto* node2 = mesh_.getNode(segment.nodeId2);
    DiametralCircle2D circle = createDiametralCircle(node1->getCoordinates(), node2->getCoordinates());
    return isPointInDiametralCircle(circle, point);
}

std::optional<Point2D> Computer2D::computeCircumcenter(const TriangleElement& element) const
{
    auto circumcircle = computeCircumcircle(element);
    if (!circumcircle.has_value())
    {
        return std::nullopt;
    }
    return circumcircle->center;
}

std::vector<size_t> Computer2D::getTrianglesSortedByQuality() const
{
    std::vector<std::pair<size_t, double>> trianglesWithQuality;

    for (const auto& [elementId, element] : mesh_.getElements())
    {
        const auto* triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (triangle == nullptr)
        {
            continue;
        }

        auto ratio = computeCircumradiusToShortestEdgeRatio(*triangle);
        double quality = ratio.has_value() ? ratio.value() : std::numeric_limits<double>::max();
        trianglesWithQuality.emplace_back(elementId, quality);
    }

    std::sort(trianglesWithQuality.begin(), trianglesWithQuality.end(),
              [](const auto& a, const auto& b) { return a.second > b.second; });

    std::vector<size_t> result;
    result.reserve(trianglesWithQuality.size());
    for (const auto& [id, quality] : trianglesWithQuality)
    {
        result.push_back(id);
    }

    return result;
}

} // namespace Meshing
