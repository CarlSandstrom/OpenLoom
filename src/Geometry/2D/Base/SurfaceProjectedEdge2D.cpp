#include "SurfaceProjectedEdge2D.h"

#include "Geometry/3D/Base/IEdge3D.h"
#include "Geometry/3D/Base/ISurface3D.h"

namespace Geometry2D
{

SurfaceProjectedEdge2D::SurfaceProjectedEdge2D(const std::string& id,
                                               const Geometry3D::ISurface3D& surface,
                                               const Geometry3D::IEdge3D& edge3D) :
    id_(id),
    surface_(&surface),
    edge3D_(&edge3D)
{
}

Meshing::Point2D SurfaceProjectedEdge2D::getPoint(double t) const
{
    auto [tMin, tMax] = edge3D_->getParameterBounds();
    double tRaw = tMin + t * (tMax - tMin);
    return surface_->projectPoint(edge3D_->getPoint(tRaw));
}

Meshing::Point2D SurfaceProjectedEdge2D::getStartPoint() const
{
    auto [tMin, tMax] = edge3D_->getParameterBounds();
    return surface_->projectPoint(edge3D_->getPoint(tMin));
}

Meshing::Point2D SurfaceProjectedEdge2D::getEndPoint() const
{
    auto [tMin, tMax] = edge3D_->getParameterBounds();
    return surface_->projectPoint(edge3D_->getPoint(tMax));
}

double SurfaceProjectedEdge2D::getLength() const
{
    return edge3D_->getLength();
}

} // namespace Geometry2D
