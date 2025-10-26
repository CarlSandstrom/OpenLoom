#pragma once

#include "../Base/Edge.h"
#include <TopoDS_Edge.hxx>

namespace Geometry
{

/**
 * @brief OpenCASCADE implementation of Edge
 */
class OpenCascadeEdge : public Edge
{
public:
    explicit OpenCascadeEdge(const TopoDS_Edge& edge);

    std::array<double, 3> getPoint(double t) const override;
    std::array<double, 3> getTangent(double t) const override;
    std::array<double, 3> getStartPoint() const override;
    std::array<double, 3> getEndPoint() const override;
    void getParameterBounds(double& tMin, double& tMax) const override;
    double getLength() const override;
    double getCurvature(double t) const override;
    std::string getId() const override;

private:
    TopoDS_Edge edge_;
};

} // namespace Geometry