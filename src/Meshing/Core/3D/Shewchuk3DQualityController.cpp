#include "Shewchuk3DQualityController.h"

#include "ElementGeometry3D.h"
#include "ElementQuality3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

Shewchuk3DQualityController::Shewchuk3DQualityController(const MeshData3D& meshData,
                                                         double circumradiusToShortestEdgeRatioBound,
                                                         std::size_t elementLimit) :
    geometry_(std::make_unique<ElementGeometry3D>(meshData)),
    quality_(std::make_unique<ElementQuality3D>(meshData)),
    circumradiusToShortestEdgeRatioBound_(circumradiusToShortestEdgeRatioBound),
    elementLimit_(elementLimit)
{
    if (circumradiusToShortestEdgeRatioBound <= 2.0)
    {
        spdlog::warn("Shewchuk3DQualityController: B bound {} is <= 2.0. "
                     "Termination is only guaranteed for B > 2.0",
                     circumradiusToShortestEdgeRatioBound);
    }
}

bool Shewchuk3DQualityController::isMeshAcceptable(const MeshData3D& data,
                                                   const MeshConnectivity& /*connectivity*/) const
{
    // Check element count limit
    if (data.getElementCount() > elementLimit_)
    {
        spdlog::debug("Shewchuk3DQualityController: Mesh exceeds element limit ({} > {})",
                      data.getElementCount(), elementLimit_);
        return true; // Accept mesh to prevent infinite refinement
    }

    // Check quality of all tetrahedra
    for (const auto& [id, element] : data.getElements())
    {
        const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(element.get());
        if (tetrahedron == nullptr)
        {
            continue; // Skip non-tetrahedral elements
        }

        if (!isTetrahedronAcceptable(*tetrahedron))
        {
            return false;
        }
    }

    return true;
}

bool Shewchuk3DQualityController::isTetrahedronAcceptable(const TetrahedralElement& element) const
{
    double ratio = quality_->getCircumradiusToShortestEdgeRatio(element);

    // Handle degenerate cases
    if (ratio == 0.0)
    {
        spdlog::debug("Shewchuk3DQualityController: Degenerate tetrahedron detected");
        return false;
    }

    if (std::isinf(ratio))
    {
        spdlog::debug("Shewchuk3DQualityController: Tetrahedron has very small edge");
        return false;
    }

    // Check against Shewchuk's B bound
    if (ratio > circumradiusToShortestEdgeRatioBound_)
    {
        return false;
    }

    return true;
}

double Shewchuk3DQualityController::getTargetElementQuality() const
{
    return circumradiusToShortestEdgeRatioBound_;
}

std::size_t Shewchuk3DQualityController::getElementLimit() const
{
    return elementLimit_;
}

} // namespace Meshing
