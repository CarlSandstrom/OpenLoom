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
 * @brief The point-against-one-CAD-surface queries the RCDT module needs: how
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
/// other. Returns 0.0 when the point does not project onto the surface at all,
/// which reads as "on it" -- callers that need to tell the two apart should
/// use projectToSurface().
double signedDistance(const Point3D& point, const Geometry3D::ISurface3D& surface);

/// The point of @p surface nearest @p point, at any distance. Nullopt when the
/// projection fails. There is deliberately no maximum-gap guard: the
/// circumcenters of the large triangles early refinement works with are
/// legitimately far from the surface they belong to (OPE-150).
std::optional<Point3D> projectToSurface(const Point3D& point,
                                        const Geometry3D::ISurface3D& surface);

/// Finds where @p surface crosses the segment [segmentStart, segmentEnd] — the
/// restricted Voronoi vertex for a face whose dual Voronoi edge runs between
/// the two (typically the circumcenters of the two tetrahedra sharing that
/// face). Uses bisection on signedDistance(), so it works regardless of how
/// curved the surface is between the endpoints.
///
/// Returns nullopt when the two endpoints are on the same side and so there is
/// no crossing to bisect on. That is an ordinary outcome, not a caller error:
/// the restricted-face classification that asks for an insertion point is
/// decided by an independent tessellation crossing test
/// (DualEdgeRestrictionOracle), and can be a stale snapshot besides, so a face
/// may legitimately be restricted while this signed-distance test sees no
/// crossing. RestrictedTriangleRefiner handles that by falling back to
/// projecting the circumcenter.
std::optional<Point3D> findSurfaceCrossing(const Point3D& segmentStart,
                                           const Point3D& segmentEnd,
                                           const Geometry3D::ISurface3D& surface);

} // namespace SurfaceProjector

} // namespace Meshing
