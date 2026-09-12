#include "Meshing/Core/3D/RCDT/RCDTTetQualityController.h"

#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/ElementQuality3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "spdlog/spdlog.h"

#include <cmath>

namespace Meshing
{

RCDTTetQualityController::RCDTTetQualityController(const MeshData3D& meshData,
                                                   const SurfaceMesh3DQualitySettings& settings) :
    meshData_(&meshData),
    settings_(settings)
{
    if (settings_.tetCircumradiusToShortestEdgeRatio <= 2.0)
    {
        spdlog::warn("RCDTTetQualityController: tetCircumradiusToShortestEdgeRatio {} is <= 2.0. "
                     "Termination is only guaranteed for values > 2.0",
                     settings_.tetCircumradiusToShortestEdgeRatio);
    }
}

bool RCDTTetQualityController::isMeshAcceptable(const MeshData3D& data,
                                                const MeshConnectivity& /*connectivity*/) const
{
    if (data.getElementCount() > settings_.tetElementLimit)
    {
        spdlog::debug("RCDTTetQualityController: Mesh exceeds tet element limit ({} > {})",
                      data.getElementCount(), settings_.tetElementLimit);
        return true; // Accept mesh to prevent infinite refinement
    }

    for (const auto& [id, element] : data.getElements())
    {
        const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tetrahedron)
            continue;

        if (!isTetrahedronAcceptable(*tetrahedron))
            return false;
    }

    return true;
}

bool RCDTTetQualityController::isTetrahedronAcceptable(const TetrahedralElement& element) const
{
    const ElementQuality3D elementQuality(*meshData_);
    const double ratio = elementQuality.getCircumradiusToShortestEdgeRatio(element);

    if (ratio == 0.0)
        return false; // degenerate

    if (std::isinf(ratio))
        return false; // near-zero shortest edge

    return ratio <= settings_.tetCircumradiusToShortestEdgeRatio;
}

double RCDTTetQualityController::getTargetElementQuality() const
{
    return settings_.tetCircumradiusToShortestEdgeRatio;
}

std::size_t RCDTTetQualityController::getElementLimit() const
{
    return settings_.tetElementLimit;
}

bool RCDTTetQualityController::isTetrahedronTooSmall(const TetrahedralElement& element) const
{
    const ElementGeometry3D elementGeometry(*meshData_);
    if (std::abs(elementGeometry.computeVolume(element)) < MIN_REFINABLE_VOLUME)
        return true;

    const ElementQuality3D elementQuality(*meshData_);
    if (elementQuality.getShortestEdgeLength(element) < MIN_REFINABLE_EDGE)
        return true;

    return false;
}

} // namespace Meshing
