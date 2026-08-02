#pragma once

#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Interfaces/IQualityController3D.h"

#include <cstddef>
#include <memory>

namespace Meshing
{
class ElementGeometry3D;
class ElementQuality3D;
} // namespace Meshing

namespace Meshing
{

/**
 * @brief Tetrahedron quality controller for RCDTRefiner's volume-quality
 * refinement priority (priority 3, RCDTMesher::meshVolume() only).
 *
 * Same circumradius-to-shortest-edge ratio criterion as the legacy Shewchuk
 * volume mesher's quality controller (removed in OPE-161), reading its bound
 * from the unified SurfaceMesh3DQualitySettings instead of taking it as bare
 * constructor arguments, consistent with RCDTQualityController's (the
 * restricted-triangle counterpart) settings-driven construction.
 */
class RCDTTetQualityController : public IQualityController3D
{
public:
    RCDTTetQualityController(const MeshData3D& meshData, const SurfaceMesh3DQualitySettings& settings);
    ~RCDTTetQualityController() override;

    bool isMeshAcceptable(const MeshData3D& data, const MeshConnectivity& connectivity) const override;
    bool isTetrahedronAcceptable(const TetrahedralElement& element) const override;
    double getTargetElementQuality() const override;
    std::size_t getElementLimit() const override;
    bool isTetrahedronTooSmall(const TetrahedralElement& element) const override;

    // Minimum thresholds for refinable tetrahedra, matching the legacy
    // Shewchuk volume mesher's precedent values.
    static constexpr double MIN_REFINABLE_VOLUME = 1e-18;
    static constexpr double MIN_REFINABLE_EDGE = 1e-10;

private:
    std::unique_ptr<ElementGeometry3D> geometry_;
    std::unique_ptr<ElementQuality3D> quality_;
    SurfaceMesh3DQualitySettings settings_;
};

} // namespace Meshing
