#pragma once

#include <array>
#include <string>

namespace Geometry
{

/**
 * @brief Abstract interface for geometric vertices/corners
 */
class Corner
{
public:
    virtual ~Corner() = default;

    /**
     * @brief Get vertex coordinates
     * @return Point coordinates [x, y, z]
     */
    virtual std::array<double, 3> getPoint() const = 0;

    /**
     * @brief Get distance to another point
     * @param point Point coordinates [x, y, z]
     * @return Distance between points
     */
    virtual double getDistanceToPoint(const std::array<double, 3>& point) const = 0;

    /**
     * @brief Get tolerance/precision of the vertex
     * @return Tolerance value
     */
    virtual double getTolerance() const = 0;

    /**
     * @brief Check if this corner is coincident with another point within tolerance
     * @param point Point coordinates [x, y, z]
     * @param tolerance Tolerance for comparison
     * @return True if points are coincident
     */
    virtual bool isCoincident(const std::array<double, 3>& point, double tolerance) const = 0;

    /**
     * @brief Get unique identifier for this corner
     */
    virtual std::string getId() const = 0;
};

} // namespace Geometry