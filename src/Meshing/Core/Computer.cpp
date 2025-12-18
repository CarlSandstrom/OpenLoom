#include "Computer.h"

#include "Meshing/Data/TriangleElement.h"
#include <cmath>
#include <optional>

namespace Meshing
{

Computer::Computer(const MeshData& mesh) :
    mesh_(mesh)
{
}

double Computer::computeVolume(const Point3DRef v0,
                               const Point3DRef v1,
                               const Point3DRef v2,
                               const Point3DRef v3)
{
    return ElementGeometry::computeVolume(v0, v1, v2, v3);
}

double Computer::computeArea(const Point3DRef v0,
                             const Point3DRef v1,
                             const Point3DRef v2)
{
    return ElementGeometry::computeArea(v0, v1, v2);
}

std::optional<ElementGeometry::CircumscribedSphere> Computer::getCircumscribingSphere(const Point3DRef v0,
                                                                                      const Point3DRef v1,
                                                                                      const Point3DRef v2,
                                                                                      const Point3DRef v3)
{
    return ElementGeometry::computeCircumscribingSphere(v0, v1, v2, v3);
}

bool Computer::getIsPointInsideCircumscribingSphere(const ElementGeometry::CircumscribedSphere& sphere,
                                                    const Point3DRef point,
                                                    double tolerance)
{
    const double radiusSq = sphere.radius * sphere.radius;
    const double distanceSq = (point - sphere.center).squaredNorm();
    return distanceSq <= radiusSq + tolerance;
}

double Computer::computeVolume(const TetrahedralElement& element) const
{
    if (const auto maybeVolume = ElementGeometry::computeVolume(mesh_, element))
    {
        return *maybeVolume;
    }

    return 0.0;
}

double Computer::computeArea(const TriangleElement& element) const
{
    if (const auto maybeArea = ElementGeometry::computeArea(mesh_, element))
    {
        return *maybeArea;
    }

    return 0.0;
}

std::optional<ElementGeometry::CircumscribedSphere> Computer::getCircumscribingSphere(const TetrahedralElement& element) const
{
    return ElementGeometry::computeCircumscribingSphere(mesh_, element);
}

bool Computer::getIsPointInsideCircumscribingSphere(const TetrahedralElement& element, const Point3D& point) const
{
    if (const auto sphere = getCircumscribingSphere(element))
    {
        return getIsPointInsideCircumscribingSphere(*sphere, point);
    }

    return false;
}

double Computer::computeQuality(const TetrahedralElement& element) const
{
    if (const auto maybeQuality = ElementGeometry::computeQuality(mesh_, element))
    {
        return *maybeQuality;
    }

    return 0.0;
}

double Computer::computeQuality(const TriangleElement& element) const
{
    if (const auto maybeQuality = ElementGeometry::computeQuality(mesh_, element))
    {
        return *maybeQuality;
    }

    return 0.0;
}

double Computer::getShortestEdgeLength(const TetrahedralElement& element) const
{
    return ElementGeometry::computeShortestEdgeLength(mesh_, element);
}

double Computer::getCircumradiusToShortestEdgeRatio(const TetrahedralElement& element) const
{
    return ElementGeometry::computeCircumradiusToShortestEdgeRatio(mesh_, element);
}

bool Computer::isSkinny(const TetrahedralElement& element, double threshold) const
{
    return ElementGeometry::isSkinny(mesh_, element, threshold);
}

std::optional<Computer::CircumCircle2D> Computer::computeCircumcircle(
    const TriangleElement& tri,
    const std::unordered_map<size_t, Point2D>& nodeCoords)
{
    const auto& nodes = tri.getNodeIdArray();

    auto it0 = nodeCoords.find(nodes[0]);
    auto it1 = nodeCoords.find(nodes[1]);
    auto it2 = nodeCoords.find(nodes[2]);

    if (it0 == nodeCoords.end() || it1 == nodeCoords.end() || it2 == nodeCoords.end())
    {
        return std::nullopt;
    }

    const Point2D& p0 = it0->second;
    const Point2D& p1 = it1->second;
    const Point2D& p2 = it2->second;

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
    circle.radiusSquared = cx * cx + cy * cy;

    return circle;
}

bool Computer::isPointInsideCircumcircle(const CircumCircle2D& circle, const Point2D& point)
{
    const double dx = point.x() - circle.center.x();
    const double dy = point.y() - circle.center.y();
    const double distSquared = dx * dx + dy * dy;

    return distSquared < circle.radiusSquared - 1e-10;
}

} // namespace Meshing
