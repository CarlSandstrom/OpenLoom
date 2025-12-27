#pragma once

#include "../Base/IEdge2D.h"
#include <Geom2d_Curve.hxx>

namespace Geometry2D
{

/**
 * @brief OpenCASCADE implementation of 2D Edge in parametric space
 *
 * Represents a curve in 2D parametric (u,v) space using OpenCASCADE's Geom2d_Curve.
 */
class OpenCascade2DEdge : public IEdge2D
{
public:
    explicit OpenCascade2DEdge(const Handle(Geom2d_Curve)& curve,
                               const std::string& id = "");

    Meshing::Point2D getPoint(double t) const override;
    Meshing::Point2D getStartPoint() const override;
    Meshing::Point2D getEndPoint() const override;
    std::pair<double, double> getParameterBounds() const override;
    double getLength() const override;

    std::string getId() const override;

private:
    Handle(Geom2d_Curve) curve_;
    std::string id_;
    double firstParam_;
    double lastParam_;
};

} // namespace Geometry2D
