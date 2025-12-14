#pragma once

#include <string>
#include <utility>

#include "Common/Types.h"

namespace Geometry2D
{

/**
 * @brief Abstract interface for geometric edges/curves in 2D parametric space
 *
 * Represents a curve in the (u,v) parametric domain of a surface.
 */
class IEdge2D
{
public:
    virtual ~IEdge2D() = default;

    virtual Meshing::Point2D getPoint(double t) const = 0;
    virtual Meshing::Point2D getStartPoint() const = 0;
    virtual Meshing::Point2D getEndPoint() const = 0;
    virtual std::pair<double, double> getParameterBounds() const = 0;
    virtual double getLength() const = 0;

    virtual std::string getId() const = 0;
};

} // namespace Geometry2D
