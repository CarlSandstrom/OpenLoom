#pragma once

#include <array>
#include <string>

#include "Common/BoundingBox2D.h"
#include "Common/Types.h"

namespace Geometry3D
{

/**
 * @brief Abstract interface for geometric surfaces
 *
 * Provides methods for querying surface properties needed for meshing
 */
class ISurface3D
{
public:
    virtual ~ISurface3D() = default;

    virtual std::array<double, 3> getNormal(double u, double v) const = 0;
    virtual Meshing::Point3D getPoint(double u, double v) const = 0;
    virtual Common::BoundingBox2D getParameterBounds() const = 0;
    virtual double getGap(const Meshing::Point3D& point) const = 0;
    virtual Meshing::Point2D projectPoint(const Meshing::Point3D& point) const = 0;

    virtual std::string getId() const = 0;
};

} // namespace Geometry3D