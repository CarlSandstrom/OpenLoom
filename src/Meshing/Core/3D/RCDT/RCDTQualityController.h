#pragma once

#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"

#include <cstddef>
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
 * @brief Decides whether a restricted surface triangle meets quality criteria.
 *
 * Ambient-space counterpart to SurfaceMeshQualityController: the same four
 * criteria (circumradius/shortest-edge ratio, minimum angle, chord deviation,
 * element limit), but evaluated directly on MeshData3D coordinates rather
 * than UV coordinates lifted via ISurface3D::getPoint(), and resolving the
 * CAD surface per triangle by surfaceId rather than being bound to one face.
 */
class RCDTQualityController
{
public:
    RCDTQualityController(const MeshData3D& meshData,
                          const Geometry3D::GeometryCollection3D& geometry,
                          const SurfaceMesh3DQualitySettings& settings);

    /// True once the restricted-triangulation size has reached the element limit —
    /// refinement should stop growing the mesh further at that point.
    bool isMeshAcceptable(std::size_t restrictedFaceCount) const;

    bool isTriangleAcceptable(const TriangleElement& triangle, const std::string& surfaceId) const;

    std::size_t getElementLimit() const { return settings_.elementLimit; }

private:
    const MeshData3D* meshData_;
    const Geometry3D::GeometryCollection3D* geometry_;
    SurfaceMesh3DQualitySettings settings_;
    SurfaceProjector surfaceProjector_;
};

} // namespace Meshing
