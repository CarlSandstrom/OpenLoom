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

    /**
     * @brief Get the surface normal at given parameter coordinates
     * @param u Parameter coordinate in u direction
     * @param v Parameter coordinate in v direction
     * @return Normal vector [x, y, z]
     */
    virtual std::array<double, 3> getNormal(double u, double v) const = 0;

    /**
     * @brief Get point on surface at given parameter coordinates
     * @param u Parameter coordinate in u direction
     * @param v Parameter coordinate in v direction
     * @return Point coordinates [x, y, z]
     */
    virtual std::array<double, 3> getPoint(double u, double v) const = 0;

    /**
     * @brief Get parameter bounds of the surface
     * @param uMin Minimum u parameter
     * @param uMax Maximum u parameter
     * @param vMin Minimum v parameter
     * @param vMax Maximum v parameter
     */
    virtual void getParameterBounds(double& uMin, double& uMax,
                                    double& vMin, double& vMax) const = 0;

    /**
     * @brief Get surface area
     * @return Surface area
     */
    virtual double getArea() const = 0;

    /**
     * @brief Get surface curvature at parameter coordinates
     * @param u Parameter coordinate in u direction
     * @param v Parameter coordinate in v direction
     * @param gaussianCurvature Output Gaussian curvature
     * @param meanCurvature Output mean curvature
     */
    virtual void getCurvature(double u, double v,
                              double& gaussianCurvature,
                              double& meanCurvature) const = 0;

    /**
     * @brief Get unique identifier for this surface
     */
    virtual std::string getId() const = 0;
};

} // namespace Geometry