#pragma once

#include <optional>
#include <string>

#include "Common/BoundingBox2D.h"
#include "Common/Types.h"

namespace Geometry3D
{

/**
 * @brief The two principal curvatures at a point on a surface.
 *
 * Signed with respect to the normal returned by ISurface3D::getNormal() at
 * the same point: positive means the surface curves TOWARD that normal (the
 * normal points to the concave side), negative means it curves away. A
 * sphere carrying an outward normal therefore has both curvatures equal to
 * -1/radius, and a cylinder has {-1/radius, 0}.
 *
 * minimum <= maximum always. Element size is bounded by the largest
 * magnitude, std::max(|minimum|, |maximum|) -- sizing cares how sharply the
 * surface bends, not which way, so it should take absolute values.
 */
struct PrincipalCurvatures
{
    double minimum = 0.0;
    double maximum = 0.0;
};

/**
 * @brief A surface point and its two first partial derivatives at one
 * parameter point (u, v).
 */
struct SurfaceDerivatives
{
    Meshing::Point3D point;
    Meshing::Vector3D derivativeU;
    Meshing::Vector3D derivativeV;
};

/**
 * @brief Abstract interface for geometric surfaces
 *
 * Provides methods for querying surface properties needed for meshing
 */
class ISurface3D
{
public:
    virtual ~ISurface3D() = default;

    /// Unit normal at (u, v); nullopt where it is not defined (a parametric
    /// pole, a cone apex, a degenerate patch).
    virtual std::optional<Meshing::Vector3D> getNormal(double u, double v) const = 0;
    virtual Meshing::Point3D getPoint(double u, double v) const = 0;
    virtual SurfaceDerivatives getFirstDerivatives(double u, double v) const = 0;
    virtual Common::BoundingBox2D getParameterBounds() const = 0;
    /// Distance from point to the surface; nullopt when it cannot be projected.
    virtual std::optional<double> getGap(const Meshing::Point3D& point) const = 0;
    virtual std::optional<Meshing::Point2D> projectPointToUnderlyingSurface(
        const Meshing::Point3D& point) const = 0;
    virtual std::optional<Meshing::Point2D> projectPointToUnderlyingSurface(
        const Meshing::Point3D& point,
        const Meshing::Point2D& seedUV) const = 0;

    virtual std::string getId() const = 0;

    /**
     * @brief Principal curvatures at parameter point (u, v)
     *
     * Surface counterpart to IEdge3D::getCurvature(). Returns nullopt where
     * curvature is not defined -- a parametric pole or cone apex, a
     * degenerate patch, or an implementation that has no curvature
     * information at all.
     *
     * nullopt is deliberately distinct from a flat {0, 0}: a caller deriving
     * element size from curvature must be able to tell "this surface is
     * planar here" (no size constraint) from "curvature is unknown here"
     * (no information, so no claim either way).
     *
     * Default returns nullopt, so implementations without curvature support
     * (e.g. simple analytic surfaces used in tests) need not provide it.
     */
    virtual std::optional<PrincipalCurvatures> getPrincipalCurvatures(double u, double v) const;

    /**
     * @brief Whether UV parameter point (u, v) lies within this surface's
     * trimmed boundary.
     *
     * The underlying math surface (used by getPoint/getNormal/
     * projectPointToUnderlyingSurface) is generally unbounded or periodic;
     * the trimmed boundary is the actual finite patch the surface occupies
     * on the model. A point can lie exactly on the untrimmed surface while
     * sitting well outside its trimmed patch -- e.g. past the edge where it
     * meets a neighboring surface, or inside a hole.
     *
     * Default is true: surfaces with no independent trim concept (e.g.
     * simple analytic surfaces used in tests) are trimmed nowhere.
     */
    virtual bool isUVWithinTrimmedBoundary(double u, double v) const;
};

} // namespace Geometry3D