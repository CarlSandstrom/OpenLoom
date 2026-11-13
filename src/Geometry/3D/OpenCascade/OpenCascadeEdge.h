#pragma once

#include "Geometry/3D/Base/IEdge3D.h"
#include <TopoDS_Edge.hxx>

namespace Geometry3D
{

/**
 * @brief OpenCASCADE implementation of IEdge3D, backed by a TopoDS_Edge
 */
class OpenCascadeEdge : public IEdge3D
{
public:
    explicit OpenCascadeEdge(const TopoDS_Edge& edge);

    Meshing::Point3D getPoint(double parameter) const override;
    Meshing::Vector3D getTangent(double parameter) const override;
    Meshing::Point3D getStartPoint() const override;
    Meshing::Point3D getEndPoint() const override;
    std::pair<double, double> getParameterBounds() const override;
    double getLength() const override;
    double getParameterAtArcLengthFraction(double parameterStart, double parameterEnd, double fraction) const override;
    double getCurvature(double parameter) const override;
    std::string getId() const override;
    bool isDegenerate() const override;

private:
    TopoDS_Edge edge_;
};

} // namespace Geometry3D