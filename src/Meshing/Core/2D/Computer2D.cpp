#include "Computer2D.h"
#include "GeometryUtilities2D.h"

namespace Meshing
{

Computer2D::Computer2D(const MeshData2D& mesh) :
    mesh_(mesh),
    geometry_(mesh),
    quality_(mesh),
    constraints_(mesh)
{
}

// ElementGeometry2D delegations
std::optional<CircumCircle2D> Computer2D::computeCircumcircle(const TriangleElement& tri) const
{
    return geometry_.computeCircumcircle(tri);
}

double Computer2D::computeArea(const TriangleElement& element) const
{
    return geometry_.computeArea(element);
}

std::array<double, 3> Computer2D::computeTriangleAngles(const TriangleElement& element) const
{
    return geometry_.computeTriangleAngles(element);
}

double Computer2D::computeMinAngle(const TriangleElement& element) const
{
    return geometry_.computeMinAngle(element);
}

std::optional<Point2D> Computer2D::computeCircumcenter(const TriangleElement& element) const
{
    return geometry_.computeCircumcenter(element);
}

// ElementQuality2D delegations
double Computer2D::computeShortestEdgeLength(const TriangleElement& element) const
{
    return quality_.computeShortestEdgeLength(element);
}

double Computer2D::computeLongestEdgeLength(const TriangleElement& element) const
{
    return quality_.computeLongestEdgeLength(element);
}

std::optional<double> Computer2D::computeCircumradiusToShortestEdgeRatio(const TriangleElement& element) const
{
    return quality_.computeCircumradiusToShortestEdgeRatio(element);
}

std::vector<size_t> Computer2D::getTrianglesSortedByQuality() const
{
    return quality_.getTrianglesSortedByQuality();
}

// ConstraintChecker2D delegations
bool Computer2D::isSegmentEncroached(const ConstrainedSegment2D& segment, const Point2D& point) const
{
    return constraints_.isSegmentEncroached(segment, point);
}

// GeometryUtilities2D delegations (static methods)
std::array<Point2D, 3> Computer2D::createSuperTriangle(const std::vector<Point2D>& points)
{
    return GeometryUtilities2D::createSuperTriangle(points);
}

bool Computer2D::isPointInsideCircumcircle(const CircumCircle2D& circle, const Point2D& point)
{
    return GeometryUtilities2D::isPointInsideCircumcircle(circle, point);
}

double Computer2D::computeEdgeLength(const Point2D& p1, const Point2D& p2)
{
    return GeometryUtilities2D::computeEdgeLength(p1, p2);
}

DiametralCircle2D Computer2D::createDiametralCircle(const Point2D& p1, const Point2D& p2)
{
    return GeometryUtilities2D::createDiametralCircle(p1, p2);
}

bool Computer2D::isPointInDiametralCircle(const DiametralCircle2D& circle, const Point2D& point)
{
    return GeometryUtilities2D::isPointInDiametralCircle(circle, point);
}

} // namespace Meshing
