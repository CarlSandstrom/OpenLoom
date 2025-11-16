#include "Computer.h"

#include <optional>

namespace Meshing
{

Computer::Computer(const MeshData& mesh) :
    mesh_(mesh)
{
}

double Computer::computeVolume(const TetrahedralElement& element) const
{
    if (const auto maybeVolume = ElementGeometry::computeVolume(mesh_, element))
    {
        return *maybeVolume;
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
        const double radiusSq = sphere->radius * sphere->radius;
        const double distanceSq = (point - sphere->center).squaredNorm();
        const double tolerance = 1e-12;
        return distanceSq <= radiusSq + tolerance;
    }

    return false;
}

} // namespace Meshing
