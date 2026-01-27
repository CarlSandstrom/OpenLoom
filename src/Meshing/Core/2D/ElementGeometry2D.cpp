#include "ElementGeometry2D.h"
#include "GeometryUtilities2D.h"

#include <algorithm>
#include <cmath>

namespace Meshing
{

ElementGeometry2D::ElementGeometry2D(const MeshData2D& mesh) :
    mesh_(mesh)
{
}

std::optional<Circle2D> ElementGeometry2D::computeCircumcircle(const TriangleElement& tri) const
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

    Circle2D circle;
    circle.center = Point2D(p0.x() + cx, p0.y() + cy);
    circle.radius = std::sqrt(cx * cx + cy * cy);

    return circle;
}

std::optional<Point2D> ElementGeometry2D::computeCircumcenter(const TriangleElement& element) const
{
    auto circumcircle = computeCircumcircle(element);
    if (!circumcircle.has_value())
    {
        return std::nullopt;
    }
    return circumcircle->center;
}

double ElementGeometry2D::computeArea(const TriangleElement& element) const
{
    auto [v0, v1, v2] = getElementNodeCoordinates(element);
    const double dx1 = v1.x() - v0.x();
    const double dy1 = v1.y() - v0.y();
    const double dx2 = v2.x() - v0.x();
    const double dy2 = v2.y() - v0.y();

    // Cross product in 2D gives the area
    return 0.5 * std::abs(dx1 * dy2 - dy1 * dx2);
}

std::array<double, 3> ElementGeometry2D::computeTriangleAngles(const TriangleElement& element) const
{
    auto [p0, p1, p2] = getElementNodeCoordinates(element);

    const double a = GeometryUtilities2D::computeEdgeLength(p1, p2);
    const double b = GeometryUtilities2D::computeEdgeLength(p0, p2);
    const double c = GeometryUtilities2D::computeEdgeLength(p0, p1);

    const double angle0 = std::acos((b * b + c * c - a * a) / (2.0 * b * c));
    const double angle1 = std::acos((a * a + c * c - b * b) / (2.0 * a * c));
    const double angle2 = std::acos((a * a + b * b - c * c) / (2.0 * a * b));

    return {angle0, angle1, angle2};
}

double ElementGeometry2D::computeMinAngle(const TriangleElement& element) const
{
    auto angles = computeTriangleAngles(element);
    return std::min({angles[0], angles[1], angles[2]});
}

std::tuple<Point2D, Point2D, Point2D> ElementGeometry2D::getElementNodeCoordinates(const TriangleElement& element) const
{
    auto nodeIds = element.getNodeIds();
    const auto* n0 = mesh_.getNode(nodeIds[0]);
    const auto* n1 = mesh_.getNode(nodeIds[1]);
    const auto* n2 = mesh_.getNode(nodeIds[2]);
    return {n0->getCoordinates(), n1->getCoordinates(), n2->getCoordinates()};
}

} // namespace Meshing
