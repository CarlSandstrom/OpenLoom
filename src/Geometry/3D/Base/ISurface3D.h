#pragma once

#include <array>
#include <optional>
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
    virtual std::optional<Meshing::Point2D> projectPointToUnderlyingSurface(
        const Meshing::Point3D& point) const = 0;
    virtual std::optional<Meshing::Point2D> projectPointToUnderlyingSurface(
        const Meshing::Point3D& point,
        const Meshing::Point2D& seedUV) const = 0;

    virtual std::string getId() const = 0;

    /**
     * @brief Whether a 3D point lies within this surface's trimmed boundary
     *
     * The underlying math surface (used by getPoint/getNormal/
     * projectPointToUnderlyingSurface) is generally unbounded or periodic;
     * the trimmed boundary is the actual finite patch the surface occupies
     * on the model. A point can lie exactly on the untrimmed surface (zero
     * gap) while sitting well outside its trimmed patch -- e.g. past the
     * edge where it meets a neighboring surface.
     *
     * Default is true: surfaces with no independent trim concept (e.g.
     * simple analytic surfaces used in tests) are trimmed nowhere.
     */
    virtual bool isPointWithinTrimmedBoundary(const Meshing::Point3D& point) const;

    /**
     * @brief Whether UV parameter point (u, v) lies within this surface's
     * trimmed boundary.
     *
     * Equivalent to isPointWithinTrimmedBoundary(getPoint(u, v)) but avoids
     * re-projecting the 3D point back to UV when the caller already has
     * the UV coordinates. Implementations that can classify in UV space
     * directly (e.g. OCC's 2D face classifier) should override this.
     *
     * Default falls back to isPointWithinTrimmedBoundary(getPoint(u, v)).
     */
    virtual bool isUVWithinTrimmedBoundary(double u, double v) const;
};

} // namespace Geometry3D