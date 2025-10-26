#pragma once

#include <array>
#include <string>

namespace Geometry
{

/**
 * @brief Abstract interface for geometric edges/curves
 */
class Edge
{
public:
    virtual ~Edge() = default;

    virtual std::array<double, 3> getPoint(double t) const = 0;
    virtual std::array<double, 3> getTangent(double t) const = 0;
    virtual std::array<double, 3> getStartPoint() const = 0;
    virtual std::array<double, 3> getEndPoint() const = 0;
    virtual void getParameterBounds(double& tMin, double& tMax) const = 0;
    virtual double getLength() const = 0;
    virtual double getCurvature(double t) const = 0;

    virtual std::string getId() const = 0;
};

} // namespace Geometry