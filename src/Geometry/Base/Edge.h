#pragma once

#include <array>
#include <string>

#include "Common/Types.h"

namespace Geometry
{

/**
 * @brief Abstract interface for geometric edges/curves
 */
class Edge
{
public:
    virtual ~Edge() = default;

    virtual Meshing::Point3D getPoint(double t) const = 0;
    virtual std::array<double, 3> getTangent(double t) const = 0;
    virtual Meshing::Point3D getStartPoint() const = 0;
    virtual Meshing::Point3D getEndPoint() const = 0;
    virtual void getParameterBounds(double& tMin, double& tMax) const = 0;
    virtual double getLength() const = 0;
    virtual double getCurvature(double t) const = 0;

    virtual std::string getId() const = 0;
};

} // namespace Geometry