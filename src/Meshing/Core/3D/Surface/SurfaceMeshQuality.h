#pragma once

#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Interfaces/IQualityController2D.h"
#include <cstddef>

namespace Meshing
{

class MeshData2D;
class TriangleElement;

/**
 * @brief Quality controller for UV-space triangles that measures quality in 3D.
 *
 * Replaces the flat UV-distance metric of Shewchuk2DQualityController with actual
 * 3D arc-length distances. For each triangle, the three UV-space vertices are lifted
 * to 3D via ISurface3D::getPoint(), and quality (circumradius-to-shortest-edge ratio,
 * minimum angle) is computed from the resulting 3D triangle geometry.
 *
 * This corrects the distortion introduced by the surface parametrisation and is
 * important for tightly curved faces where UV distances differ significantly from
 * 3D arc-lengths.
 *
 * If chordDeviationTolerance > 0 (S2.4), each triangle is also rejected when the
 * chord height — the maximum 3D distance from the flat triangle to the actual CAD
 * surface, sampled at the triangle centroid and three edge midpoints — exceeds the
 * tolerance. This ensures the triangulation geometrically approximates the surface
 * to within the given tolerance regardless of triangle size.
 */
class SurfaceMeshQualityController : public IQualityController2D
{
public:
    SurfaceMeshQualityController(const MeshData2D& meshData,
                                 const Geometry3D::ISurface3D& surface,
                                 double circumradiusToShortestEdgeRatioBound,
                                 double minAngleThresholdRadians,
                                 std::size_t elementLimit,
                                 double chordDeviationTolerance = 0.0);

    bool isMeshAcceptable(const MeshData2D& data) const override;
    bool isTriangleAcceptable(const TriangleElement& element) const override;
    double getTargetElementQuality() const override;
    std::size_t getElementLimit() const override;

private:
    const MeshData2D& meshData_;
    const Geometry3D::ISurface3D& surface_;
    double circumradiusToShortestEdgeRatioBound_;
    double minAngleThreshold_;
    std::size_t elementLimit_;
    double chordDeviationTolerance_;
};

} // namespace Meshing
