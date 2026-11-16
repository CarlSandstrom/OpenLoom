#pragma once

#include "Common/Types.h"

#include <cstddef>
#include <string>
#include <vector>

namespace Geometry3D
{
class GeometryCollection3D;
class IEdge3D;
class ISurface3D;
} // namespace Geometry3D

namespace Topology3D
{
class Topology3D;
}

namespace Meshing
{

/**
 * @brief Distance from a point on a corner or curve to the nearest CAD surface
 * that shares no corner with it.
 *
 * A curve lying close to a surface it has nothing to do with -- the edge of a
 * pocket 1 mm below the face above it -- needs elements smaller than that gap.
 * This is the gap, measured from the point itself rather than between two sets
 * of samples, so it does not overstate a gap the samples straddle.
 *
 * ## Which surfaces count
 *
 * A surface counts iff it contains none of the corners the caller names: a
 * curve's two end corners, or the corner itself. That leaves out the surfaces
 * the curve bounds, and also every surface that merely touches one of its
 * ends. The latter is deliberate. A surface through the corner is at distance
 * zero there, so a plain distance would shrink toward every corner of every
 * model, where nothing is thin.
 *
 * The cost of that rule: a surface meeting the curve at its corner in a sharp
 * wedge is never measured, however close it runs along the curve.
 *
 * ## How the distance is found
 *
 * Each edge and each surface is sampled coarsely once. A query refines only
 * around the samples that could still beat the best distance found so far,
 * halving the parameter cell around the nearest point until the cell is small
 * against the distance. The boundary edges of the surfaces are searched first:
 * they lie on the surfaces, so the trimmed boundary -- where the nearest point
 * often is -- never has to be resolved on the surface grid, whose samples
 * outside the trim are simply dropped.
 *
 * Every point that lowers the result lies on an edge or inside a surface's
 * trimmed boundary, so the result is never below the true distance. A basin
 * the coarse samples miss can only make it too large.
 */
class NonIncidentSurfaceDistance
{
public:
    /// A coarse sample and the 3D reach of its parameter cell.
    struct EdgeSample
    {
        double parameter;
        Point3D point;
        double cellRadius;
    };

    struct SampledEdge
    {
        const Geometry3D::IEdge3D* edge;
        double parameterStep;
        std::vector<EdgeSample> samples;
    };

    struct SurfaceSample
    {
        Point2D parameters;
        Point3D point;
        double cellRadius;
    };

    struct SampledSurface
    {
        const Geometry3D::ISurface3D* surface;
        std::vector<std::string> cornerIds;
        std::vector<std::size_t> edgeIndices;
        Point2D parameterStep;
        std::vector<SurfaceSample> samples; // inside the trimmed boundary only
    };

    NonIncidentSurfaceDistance(const Geometry3D::GeometryCollection3D& geometry,
                               const Topology3D::Topology3D& topology);

    /// Distance from @p point to the nearest surface containing none of
    /// @p cornerIds; infinity when no surface qualifies.
    double distance(const Point3D& point, const std::vector<std::string>& cornerIds) const;

private:
    std::vector<SampledEdge> edges_;
    std::vector<SampledSurface> surfaces_;
};

} // namespace Meshing
