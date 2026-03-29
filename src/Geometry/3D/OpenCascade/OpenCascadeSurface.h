#pragma once

#include "../Base/ISurface3D.h"
#include <TopoDS_Face.hxx>

namespace Geometry3D
{

/**
 * @brief OpenCASCADE implementation of Surface
 */
class OpenCascadeSurface : public ISurface3D
{
public:
    explicit OpenCascadeSurface(const TopoDS_Face& face);

    std::array<double, 3> getNormal(double u, double v) const override;
    Meshing::Point3D getPoint(double u, double v) const override;
    Common::BoundingBox2D getParameterBounds() const override;
    double getGap(const Meshing::Point3D& point) const override;
    Meshing::Point2D projectPoint(const Meshing::Point3D& point) const override;
    std::optional<Meshing::Point2D> projectPointToUnderlyingSurface(
        const Meshing::Point3D& point) const override;
    std::optional<Meshing::Point2D> projectPointToUnderlyingSurface(
        const Meshing::Point3D& point,
        const Meshing::Point2D& seedUV) const override;
    std::string getId() const override;

private:
    TopoDS_Face face_;
};

} // namespace Geometry3D