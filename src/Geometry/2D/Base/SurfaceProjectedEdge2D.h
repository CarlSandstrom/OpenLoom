#pragma once

#include <string>
#include <utility>

#include "Common/Types.h"
#include "IEdge2D.h"

namespace Geometry3D
{
class ISurface3D;
class IEdge3D;
}

namespace Geometry2D
{

/**
 * @brief A 2D edge that evaluates by projecting a 3D arc onto a surface's UV parametric domain.
 *
 * Used for closed-circle boundary edges on non-periodic faces, where a LinearEdge2D
 * would be zero-length (start == end corner in 3D) and getPoint(t) would always return
 * the same UV coordinate. This class evaluates the 3D edge at the given normalized
 * t-parameter and projects the result to UV via the surface.
 *
 * The t-parameter is normalized to [0,1] consistent with LinearEdge2D convention.
 */
class SurfaceProjectedEdge2D : public IEdge2D
{
public:
    SurfaceProjectedEdge2D(const std::string& id,
                           const Geometry3D::ISurface3D& surface,
                           const Geometry3D::IEdge3D& edge3D);

    Meshing::Point2D getPoint(double t) const override;
    Meshing::Point2D getStartPoint() const override;
    Meshing::Point2D getEndPoint() const override;
    std::pair<double, double> getParameterBounds() const override { return {0.0, 1.0}; }
    double getLength() const override;

    std::string getId() const override { return id_; }

private:
    std::string id_;
    const Geometry3D::ISurface3D* surface_;
    const Geometry3D::IEdge3D* edge3D_;
};

} // namespace Geometry2D
