#include "ElementGeometry2D.h"
#include "PeriodicMeshData2D.h"
#include "Common/Exceptions/GeometryException.h"
#include "GeometryUtilities2D.h"
#include "Meshing/Data/2D/TriangleElement.h"

#include <algorithm>
#include <cmath>

namespace Meshing
{

ElementGeometry2D::ElementGeometry2D(const MeshData2D& mesh) :
    mesh_(mesh),
    periodicData_(nullptr)
{
}

ElementGeometry2D::ElementGeometry2D(const MeshData2D& mesh, PeriodicMeshData2D* periodicData) :
    mesh_(mesh),
    periodicData_(periodicData)
{
}

// --- shared math helpers ---

static std::optional<Circle2D> computeCircumcircleFromPoints(
    const Point2D& p0, const Point2D& p1, const Point2D& p2)
{
    const double ax = p1.x() - p0.x();
    const double ay = p1.y() - p0.y();
    const double bx = p2.x() - p0.x();
    const double by = p2.y() - p0.y();

    const double d = 2.0 * (ax * by - ay * bx);

    if (std::abs(d) < 1e-10)
        return std::nullopt;

    const double aSq = ax * ax + ay * ay;
    const double bSq = bx * bx + by * by;

    const double cx = (by * aSq - ay * bSq) / d;
    const double cy = (ax * bSq - bx * aSq) / d;

    Circle2D circle;
    circle.center = Point2D(p0.x() + cx, p0.y() + cy);
    circle.radius = std::sqrt(cx * cx + cy * cy);

    return circle;
}

// --- TriangleElement& overloads (canonical coordinates) ---

std::optional<Circle2D> ElementGeometry2D::computeCircumcircle(const TriangleElement& element) const
{
    auto [p0, p1, p2] = getElementNodeCoordinates(element);
    return computeCircumcircleFromPoints(p0, p1, p2);
}

std::optional<Point2D> ElementGeometry2D::computeCircumcenter(const TriangleElement& element) const
{
    auto circle = computeCircumcircle(element);
    if (!circle)
        return std::nullopt;
    return circle->center;
}

double ElementGeometry2D::computeArea(const TriangleElement& element) const
{
    auto [v0, v1, v2] = getElementNodeCoordinates(element);
    return std::abs(GeometryUtilities2D::computeSignedArea(v0, v1, v2));
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

Point2D ElementGeometry2D::computeCentroid(const TriangleElement& element) const
{
    auto [p0, p1, p2] = getElementNodeCoordinates(element);
    return Point2D((p0.x() + p1.x() + p2.x()) / 3.0,
                   (p0.y() + p1.y() + p2.y()) / 3.0);
}

std::tuple<Point2D, Point2D, Point2D> ElementGeometry2D::getElementNodeCoordinates(
    const TriangleElement& element) const
{
    const auto& nodeIds = element.getNodeIdArray();
    const auto* n0 = mesh_.getNode(nodeIds[0]);
    const auto* n1 = mesh_.getNode(nodeIds[1]);
    const auto* n2 = mesh_.getNode(nodeIds[2]);
    OPENLOOM_REQUIRE_NOT_NULL(n0, "triangle node 0");
    OPENLOOM_REQUIRE_NOT_NULL(n1, "triangle node 1");
    OPENLOOM_REQUIRE_NOT_NULL(n2, "triangle node 2");
    return {n0->getCoordinates(), n1->getCoordinates(), n2->getCoordinates()};
}

// --- element ID overloads (offset-aware when periodicData_ is set) ---

std::tuple<Point2D, Point2D, Point2D> ElementGeometry2D::getElementNodeCoordinates(size_t elementId) const
{
    if (periodicData_ && periodicData_->getOffsetTable().hasOffsets(elementId))
        return periodicData_->getTriangleCoordinates(elementId);

    const auto* element = dynamic_cast<const TriangleElement*>(mesh_.getElement(elementId));
    OPENLOOM_REQUIRE_NOT_NULL(element, "triangle element");
    return getElementNodeCoordinates(*element);
}

std::optional<Circle2D> ElementGeometry2D::computeCircumcircle(size_t elementId) const
{
    auto [p0, p1, p2] = getElementNodeCoordinates(elementId);
    return computeCircumcircleFromPoints(p0, p1, p2);
}

std::optional<Point2D> ElementGeometry2D::computeCircumcenter(size_t elementId) const
{
    auto circle = computeCircumcircle(elementId);
    if (!circle)
        return std::nullopt;
    return circle->center;
}

Point2D ElementGeometry2D::computeCentroid(size_t elementId) const
{
    auto [p0, p1, p2] = getElementNodeCoordinates(elementId);
    return Point2D((p0.x() + p1.x() + p2.x()) / 3.0,
                   (p0.y() + p1.y() + p2.y()) / 3.0);
}

} // namespace Meshing
