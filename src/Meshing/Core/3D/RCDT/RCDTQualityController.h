#pragma once

#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

#include <string>

namespace Meshing
{
class MeshData3D;
class TriangleElement;
} // namespace Meshing

namespace Geometry3D
{
class GeometryCollection3D;
} // namespace Geometry3D

namespace Meshing
{

/**
 * @brief Decides whether a restricted surface triangle is good enough to leave
 * alone, or has to be refined. RestrictedTriangulation::updateBadFaceEntry()
 * is the only caller.
 *
 * Three criteria, all measured on the ambient 3D coordinates MeshData3D holds
 * and all bounded by SurfaceMesh3DQualitySettings: the circumradius-to-shortest-
 * edge ratio, the minimum interior angle, and the chord deviation -- here the
 * distance from the triangle's circumcenter to the CAD surface the triangle is
 * restricted to, resolved per triangle by surfaceId rather than fixed for the
 * life of the object. elementLimit is deliberately not among them: a cap on how
 * many restricted triangles refinement may produce is a property of the set, and
 * RestrictedTriangulation::getBadTriangles() is where it is applied.
 *
 * This is a view, not a computation. It stores two pointers and a copy of the
 * settings struct, derives nothing in its constructor and caches nothing between
 * calls, which is why the caller builds one on the stack per face rather than
 * keeping one. Measured at -O0, where nothing is inlined: 42 instructions per
 * construction against ~58,000 for the isTriangleAcceptable() call that follows
 * it (callgrind on HexNutSurfaceMesh, OPE-202).
 *
 * The legacy UV-space pipeline's SurfaceMeshQualityController answers the same
 * question for its own mesher, but it is not the same test and neither is a port
 * of the other: it works in the UV coordinates of one fixed face lifted through
 * ISurface3D::getPoint(), it measures chord deviation as the gap between the flat
 * triangle and the surface at the centroid and the three edge midpoints, and it
 * rejects a degenerate triangle where this class accepts one.
 */
class RCDTQualityController
{
public:
    RCDTQualityController(const MeshData3D& meshData,
                          const Geometry3D::GeometryCollection3D& geometry,
                          const SurfaceMesh3DQualitySettings& settings);

    bool isTriangleAcceptable(const TriangleElement& triangle, const std::string& surfaceId) const;

private:
    const MeshData3D* meshData_;
    const Geometry3D::GeometryCollection3D* geometry_;
    SurfaceMesh3DQualitySettings settings_;
};

} // namespace Meshing
