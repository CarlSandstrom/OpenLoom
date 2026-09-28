#pragma once

#include "Common/Types.h"
#include "Meshing/Connectivity/FaceKey.h"
#include "Meshing/Core/3D/RCDT/SurfaceTessellation.h"

#include <optional>
#include <string>
#include <unordered_map>

namespace Geometry3D
{
class GeometryCollection3D;
} // namespace Geometry3D

namespace Topology3D
{
class Topology3D;
} // namespace Topology3D

namespace Meshing
{
class MeshConnectivity;
class MeshData3D;
} // namespace Meshing

namespace Meshing
{

/// A face of the tetrahedralization restricted to a surface, and where its
/// dual edge meets that surface: the centre of its surface Delaunay ball.
struct RestrictedFacet
{
    std::string surfaceId;
    Point3D surfaceCenter;
};

/**
 * @brief Restricted Delaunay test as CGAL Mesh_3 and JIGSAW define it: a face
 * is restricted exactly when its dual edge in the power diagram crosses a
 * surface, and belongs to the surface it crosses.
 *
 * The dual edge joins the WEIGHTED circumcenters (orthocenters) of the two
 * tetrahedra sharing the face, because the tetrahedralization is regular --
 * weighted by the protecting balls -- and in a regular triangulation that,
 * not the segment between ordinary circumcenters, is the dual.
 *
 * Deliberately nothing else: no gate on which surfaces the vertices share, no
 * shortcut routes, no inside/outside test. Where the answer is locally wrong
 * the refiner that uses this is expected to refine it away, as CGAL does. This
 * is the restriction half of the CGAL-style refinement path (OPE-186); the
 * existing DualEdgeRestrictionOracle is untouched by it.
 *
 * Crossings are found on each surface's SurfaceTessellation, then refined onto
 * the exact CAD surface by bisection along the dual segment, so the centre
 * stays on the dual line, and accepted only within the surface's trimmed
 * patch. When the dual edge crosses more than once, the
 * crossing nearest the face's own weighted circumcenter wins: that point lies
 * on the dual line, so the nearest crossing is the one belonging to this face.
 */
class WeightedDualRestriction
{
public:
    /// minimumEdgeLength sizes each surface's tessellation, the same way
    /// DualEdgeRestrictionOracle sizes its own.
    WeightedDualRestriction(const Geometry3D::GeometryCollection3D& geometry,
                            const Topology3D::Topology3D& topology,
                            double minimumEdgeLength);

    /// nullopt when face is not restricted: its dual edge crosses no surface,
    /// it has fewer than two adjacent tetrahedra, or it touches the bounding
    /// tetrahedron.
    std::optional<RestrictedFacet> restrict(const FaceKey& face,
                                            const MeshData3D& meshData,
                                            const MeshConnectivity& connectivity) const;

private:
    const Geometry3D::GeometryCollection3D* geometry_;
    double cellSize_;
    std::unordered_map<std::string, SurfaceTessellation> surfaceTessellations_;
};

} // namespace Meshing
