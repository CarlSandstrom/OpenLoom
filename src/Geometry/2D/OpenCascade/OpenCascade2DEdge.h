#pragma once

#include "Geometry/2D/Base/IEdge2D.h"
#include <Geom2d_Curve.hxx>

namespace Geometry2D
{

/**
 * @brief OpenCASCADE implementation of IEdge2D, backed by a Geom2d_Curve in (u,v) parametric space
 */
class OpenCascade2DEdge : public IEdge2D
{
public:
    explicit OpenCascade2DEdge(const Handle(Geom2d_Curve) & curve,
                               const std::string& id = "");

    Meshing::Point2D getPoint(double parameter) const override;
    Meshing::Point2D getStartPoint() const override;
    Meshing::Point2D getEndPoint() const override;
    std::pair<double, double> getParameterBounds() const override;
    double getLength() const override;

    std::string getId() const override;

private:
    Handle(Geom2d_Curve) curve_;
    std::string id_;
    double firstParameter_;
    double lastParameter_;
};

} // namespace Geometry2D
