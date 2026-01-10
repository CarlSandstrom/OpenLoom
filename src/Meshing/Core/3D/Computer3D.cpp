#include "Computer3D.h"

namespace Meshing
{

Computer3D::Computer3D(const MeshData3D& mesh) :
    mesh_(mesh),
    geometry_(mesh),
    quality_(mesh)
{
}

// ElementGeometry3D delegations
double Computer3D::computeVolume(const TetrahedralElement& element) const
{
    return geometry_.computeVolume(element);
}

double Computer3D::computeArea(const TriangleElement& element) const
{
    return geometry_.computeArea(element);
}

std::optional<CircumscribedSphere> Computer3D::computeCircumscribingSphere(const TetrahedralElement& element) const
{
    return geometry_.computeCircumscribingSphere(element);
}

bool Computer3D::getIsPointInsideCircumscribingSphere(const TetrahedralElement& element,
                                                      const Point3D& point,
                                                      double tolerance) const
{
    return geometry_.isPointInsideCircumscribingSphere(element, point, tolerance);
}

// ElementQuality3D delegations
double Computer3D::getShortestEdgeLength(const TetrahedralElement& element) const
{
    return quality_.getShortestEdgeLength(element);
}

double Computer3D::getCircumradiusToShortestEdgeRatio(const TetrahedralElement& element) const
{
    return quality_.getCircumradiusToShortestEdgeRatio(element);
}

bool Computer3D::isSkinny(const TetrahedralElement& element, double threshold) const
{
    return quality_.isSkinny(element, threshold);
}

} // namespace Meshing
