#include "Meshing/Core/2D/General/ElementGeometry2D.h"
#include "Common/Exceptions/GeometryException.h"
#include "Meshing/Core/2D/General/GeometryUtilities2D.h"
#include "Meshing/Core/2D/Periodic/PeriodicMeshData2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/Base/TriangleElement.h"

#include <algorithm>
#include <cmath>

namespace Meshing
{

namespace
{
std::optional<Circle2D> computeCircumcircleFromPoints(const Point2D& point0, const Point2D& point1,
                                                      const Point2D& point2);
Point2D computeCentroidFromPoints(const Point2D& point0, const Point2D& point1, const Point2D& point2);
} // namespace

ElementGeometry2D::ElementGeometry2D(const MeshData2D& mesh) :
    ElementGeometry2D(mesh, nullptr)
{
}

ElementGeometry2D::ElementGeometry2D(const MeshData2D& mesh, PeriodicMeshData2D* periodicData) :
    mesh_(mesh),
    periodicData_(periodicData)
{
}

std::optional<Circle2D> ElementGeometry2D::computeCircumcircle(const TriangleElement& element) const
{
    auto [point0, point1, point2] = getElementNodeCoordinates(element);
    return computeCircumcircleFromPoints(point0, point1, point2);
}

std::optional<Point2D> ElementGeometry2D::computeCircumcenter(const TriangleElement& element) const
{
    auto circle = computeCircumcircle(element);
    if (!circle) return std::nullopt;
    return circle->center;
}

double ElementGeometry2D::computeArea(const TriangleElement& element) const
{
    auto [point0, point1, point2] = getElementNodeCoordinates(element);
    return std::abs(GeometryUtilities2D::computeSignedArea(point0, point1, point2));
}

std::array<double, 3> ElementGeometry2D::computeTriangleAngles(const TriangleElement& element) const
{
    auto [point0, point1, point2] = getElementNodeCoordinates(element);

    // Law of cosines; lengthN is the side opposite pointN.
    double length0 = GeometryUtilities2D::computeEdgeLength(point1, point2);
    double length1 = GeometryUtilities2D::computeEdgeLength(point0, point2);
    double length2 = GeometryUtilities2D::computeEdgeLength(point0, point1);

    double angle0 = std::acos((length1 * length1 + length2 * length2 - length0 * length0) / (2.0 * length1 * length2));
    double angle1 = std::acos((length0 * length0 + length2 * length2 - length1 * length1) / (2.0 * length0 * length2));
    double angle2 = std::acos((length0 * length0 + length1 * length1 - length2 * length2) / (2.0 * length0 * length1));

    return {angle0, angle1, angle2};
}

double ElementGeometry2D::computeMinAngle(const TriangleElement& element) const
{
    auto angles = computeTriangleAngles(element);
    return std::min({angles[0], angles[1], angles[2]});
}

Point2D ElementGeometry2D::computeCentroid(const TriangleElement& element) const
{
    auto [point0, point1, point2] = getElementNodeCoordinates(element);
    return computeCentroidFromPoints(point0, point1, point2);
}

std::tuple<Point2D, Point2D, Point2D> ElementGeometry2D::getElementNodeCoordinates(
    const TriangleElement& element) const
{
    auto& nodeIds = element.getNodeIdArray();
    auto node0 = mesh_.getNode(nodeIds[0]);
    auto node1 = mesh_.getNode(nodeIds[1]);
    auto node2 = mesh_.getNode(nodeIds[2]);
    OPENLOOM_REQUIRE_NOT_NULL(node0, "triangle node 0");
    OPENLOOM_REQUIRE_NOT_NULL(node1, "triangle node 1");
    OPENLOOM_REQUIRE_NOT_NULL(node2, "triangle node 2");
    return {node0->getCoordinates(), node1->getCoordinates(), node2->getCoordinates()};
}

std::tuple<Point2D, Point2D, Point2D> ElementGeometry2D::getElementNodeCoordinates(ElementId elementId) const
{
    if (periodicData_ && periodicData_->getOffsetTable().hasOffsets(elementId))
        return periodicData_->getTriangleCoordinates(elementId);

    auto element = dynamic_cast<const TriangleElement*>(mesh_.getElement(elementId));
    OPENLOOM_REQUIRE_NOT_NULL(element, "triangle element");
    return getElementNodeCoordinates(*element);
}

std::optional<Circle2D> ElementGeometry2D::computeCircumcircle(ElementId elementId) const
{
    auto [point0, point1, point2] = getElementNodeCoordinates(elementId);
    return computeCircumcircleFromPoints(point0, point1, point2);
}

std::optional<Point2D> ElementGeometry2D::computeCircumcenter(ElementId elementId) const
{
    auto circle = computeCircumcircle(elementId);
    if (!circle) return std::nullopt;
    return circle->center;
}

Point2D ElementGeometry2D::computeCentroid(ElementId elementId) const
{
    auto [point0, point1, point2] = getElementNodeCoordinates(elementId);
    return computeCentroidFromPoints(point0, point1, point2);
}

namespace
{

std::optional<Circle2D> computeCircumcircleFromPoints(const Point2D& point0, const Point2D& point1,
                                                      const Point2D& point2)
{
    double offset1X = point1.x() - point0.x();
    double offset1Y = point1.y() - point0.y();
    double offset2X = point2.x() - point0.x();
    double offset2Y = point2.y() - point0.y();

    double denominator = 2.0 * (offset1X * offset2Y - offset1Y * offset2X);

    if (std::abs(denominator) < 1e-10) return std::nullopt;

    double offset1SquaredLength = offset1X * offset1X + offset1Y * offset1Y;
    double offset2SquaredLength = offset2X * offset2X + offset2Y * offset2Y;

    double centerOffsetX = (offset2Y * offset1SquaredLength - offset1Y * offset2SquaredLength) / denominator;
    double centerOffsetY = (offset1X * offset2SquaredLength - offset2X * offset1SquaredLength) / denominator;

    Circle2D circle;
    circle.center = Point2D(point0.x() + centerOffsetX, point0.y() + centerOffsetY);
    circle.radius = std::sqrt(centerOffsetX * centerOffsetX + centerOffsetY * centerOffsetY);

    return circle;
}

Point2D computeCentroidFromPoints(const Point2D& point0, const Point2D& point1, const Point2D& point2)
{
    return Point2D((point0.x() + point1.x() + point2.x()) / 3.0, (point0.y() + point1.y() + point2.y()) / 3.0);
}

} // namespace

} // namespace Meshing
