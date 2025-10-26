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

    /**
     * @brief Get point on edge at parameter t
     * @param t Parameter (typically 0 to 1)
     * @return Point coordinates [x, y, z]
     */
    virtual std::array<double, 3> getPoint(double t) const = 0;

    /**
     * @brief Get tangent vector at parameter t
     * @param t Parameter (typically 0 to 1)
     * @return Tangent vector [x, y, z]
     */
    virtual std::array<double, 3> getTangent(double t) const = 0;

    /**
     * @brief Get start point of the edge
     * @return Start point coordinates [x, y, z]
     */
    virtual std::array<double, 3> getStartPoint() const = 0;

    /**
     * @brief Get end point of the edge
     * @return End point coordinates [x, y, z]
     */
    virtual std::array<double, 3> getEndPoint() const = 0;

    /**
     * @brief Get parameter bounds
     * @param tMin Minimum parameter value
     * @param tMax Maximum parameter value
     */
    virtual void getParameterBounds(double& tMin, double& tMax) const = 0;

    /**
     * @brief Get edge length
     * @return Edge length
     */
    virtual double getLength() const = 0;

    /**
     * @brief Get curvature at parameter t
     * @param t Parameter
     * @return Curvature value
     */
    virtual double getCurvature(double t) const = 0;

    /**
     * @brief Get distance from a point to the edge
     * @param point Point coordinates [x, y, z]
     * @return Minimum distance to edge
     */
    virtual double getDistanceToPoint(const std::array<double, 3>& point) const = 0;

    /**
     * @brief Get unique identifier for this edge
     */
    virtual std::string getId() const = 0;
};

} // namespace Geometry