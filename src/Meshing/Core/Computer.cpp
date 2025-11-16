#include "Computer.h"

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

} // namespace Meshing
