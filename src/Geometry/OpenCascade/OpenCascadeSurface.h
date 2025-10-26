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
    std::array<double, 3> getPoint(double u, double v) const override;
    void getParameterBounds(double& uMin, double& uMax,
                            double& vMin, double& vMax) const override;
    double getArea() const override;
    void getCurvature(double u, double v,
                      double& gaussianCurvature,
                      double& meanCurvature) const override;
    std::string getId() const override;

private:
    TopoDS_Face face_;
};

} // namespace Geometry