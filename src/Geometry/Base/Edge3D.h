#pragma once

#include <array>
#include <string>
#include <utility>

#include "Common/Types.h"

namespace Geometry3D
{

/**
 * @brief Abstract interface for geometric edges/curves
 */
class Edge3D
{
public:
    virtual ~Edge3D() = default;

    virtual Meshing::Point3D getPoint(double t) const = 0;
    virtual std::array<double, 3> getTangent(double t) const = 0;
    virtual Meshing::Point3D getStartPoint() const = 0;
    virtual Meshing::Point3D getEndPoint() const = 0;
    virtual std::pair<double, double> getParameterBounds() const = 0;
    virtual double getLength() const = 0;
    virtual double getCurvature(double t) const = 0;

    virtual std::string getId() const = 0;
};

} // namespace Geometry3D