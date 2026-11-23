#pragma once

#include "Common/Types.h"

#include <optional>

namespace Geometry3D
{
class ISurface3D;
} // namespace Geometry3D

namespace Meshing
{

/**
 * @brief The point-against-one-CAD-surface queries a surface mesher needs: how
 * far off a surface a point lies, where it lands when pulled onto that
 * surface, and where a segment crosses it.
 *
 * Free functions rather than a class, on the same grounds as
 * RestrictedFaceAudit: every query is a pure function of a point and a
 * surface, so there is no state worth owning and nothing to configure. The
 * class this replaced had no members, yet three classes held one anyway and
 * two more built a throwaway per call -- five call sites disagreeing about the
 * lifetime of something with no lifetime.
 *
 * Everything here goes through ISurface3D, so it is not tied to OpenCASCADE,
 * but every call does reach a CAD projection: treat them as expensive.
 */
namespace SurfaceProjector
{

/// Distance from @p point to @p surface along the surface normal at its
/// projection: positive on the side the normal points to, negative on the
/// other. nullopt when the point does not project onto the surface, or the
/// normal is not defined where it does.
std::optional<double> signedDistance(const Point3D& point, const Geometry3D::ISurface3D& surface);

/// The point of @p surface nearest @p point, at any distance. Nullopt when the
/// projection fails. There is deliberately no maximum-gap guard: the
/// circumcenters of the large triangles early refinement works with are
/// legitimately far from the surface they belong to (OPE-150).
std::optional<Point3D> projectToSurface(const Point3D& point,
                                        const Geometry3D::ISurface3D& surface);

/// Finds where @p surface crosses the segment [segmentStart, segmentEnd] — the
/// restricted Voronoi vertex for a face whose dual Voronoi edge runs between
/// the two. Solves segmentStart + t (segmentEnd - segmentStart) = S(u, v) by
/// Newton's method in (t, u, v) from @p seedUV, so the answer lies on the
/// segment, and costs a few surface evaluations rather than CAD projections.
///
/// Returns nullopt when Newton does not converge, the segment is tangent to
/// the surface, or the crossing it converges to lies off the segment. That is
/// an ordinary outcome, not a caller error: restriction is decided by an
/// independent tessellation crossing test, and WeightedDualRestriction then
/// keeps the tessellation's own crossing.
std::optional<Point3D> findSurfaceCrossing(const Point3D& segmentStart,
                                           const Point3D& segmentEnd,
                                           const Geometry3D::ISurface3D& surface,
                                           const Point2D& seedUV);

} // namespace SurfaceProjector

} // namespace Meshing
