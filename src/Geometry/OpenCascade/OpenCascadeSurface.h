#pragma once

#include "../Base/Surface.h"
#include <TopoDS_Face.hxx>

namespace Geometry
{

/**
 * @brief OpenCASCADE implementation of Surface
 */
class OpenCascadeSurface : public Surface
{
public:
    explicit OpenCascadeSurface(const TopoDS_Face& face);

    std::array<double, 3> getNormal(double u, double v) const override;
    Meshing::Point3D getPoint(double u, double v) const override;
    void getParameterBounds(double& uMin, double& uMax,
                            double& vMin, double& vMax) const override;
    double getGap(const Meshing::Point3D& point) const override;
    Meshing::Point2D projectPoint(const Meshing::Point3D& point) const override;
    std::string getId() const override;

private:
    TopoDS_Face face_;
};

} // namespace Geometry