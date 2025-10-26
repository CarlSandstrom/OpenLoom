#pragma once

#include <array>
#include <string>

namespace Geometry
{

/**
 * @brief Abstract interface for geometric surfaces
 *
 * Provides methods for querying surface properties needed for meshing
 */
class Surface
{
public:
    virtual ~Surface() = default;

    virtual std::array<double, 3> getNormal(double u, double v) const = 0;
    virtual std::array<double, 3> getPoint(double u, double v) const = 0;
    virtual void getParameterBounds(double& uMin, double& uMax,
                                    double& vMin, double& vMax) const = 0;
    virtual double getGap(const std::array<double, 3>& point) const = 0;
    virtual std::array<double, 2> projectPoint(const std::array<double, 3>& point) const = 0;

    virtual std::string getId() const = 0;
};

} // namespace Geometry