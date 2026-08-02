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
class IEdge3D
{
public:
    virtual ~IEdge3D() = default;

    virtual Meshing::Point3D getPoint(double t) const = 0;
    virtual std::array<double, 3> getTangent(double t) const = 0;
    virtual Meshing::Point3D getStartPoint() const = 0;
    virtual Meshing::Point3D getEndPoint() const = 0;
    virtual std::pair<double, double> getParameterBounds() const = 0;
    virtual double getLength() const = 0;
    virtual double getParameterAtArcLengthFraction(double tStart, double tEnd, double fraction) const = 0;
    virtual double getCurvature(double t) const = 0;

    virtual std::string getId() const = 0;

    /// True for a topological placeholder edge with no real 3D geometry —
    /// e.g. a sphere's or cone's polar edge, collapsed to a single point,
    /// present only to close the surface's UV-space wire at a parametric
    /// singularity. getPoint()/getTangent() are not meaningful for such an
    /// edge. Defaults to false; only CAD-backed implementations can be
    /// degenerate.
    virtual bool isDegenerate() const { return false; }
};

} // namespace Geometry3D