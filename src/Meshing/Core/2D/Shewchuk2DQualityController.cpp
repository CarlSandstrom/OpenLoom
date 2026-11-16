#include "Shewchuk2DQualityController.h"

#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"

namespace Meshing
{

Shewchuk2DQualityController::Shewchuk2DQualityController(const MeshData2D& meshData,
                                                         double circumradiusToShortestEdgeRatioBound,
                                                         double minimumAngleThresholdRadians,
                                                         std::size_t elementLimit) :
    geometry_(std::make_unique<ElementGeometry2D>(meshData)),
    quality_(std::make_unique<ElementQuality2D>(meshData)),
    circumradiusToShortestEdgeRatioBound_(circumradiusToShortestEdgeRatioBound),
    minimumAngleThreshold_(minimumAngleThresholdRadians),
    elementLimit_(elementLimit)
{
}

bool Shewchuk2DQualityController::isMeshAcceptable(const MeshData2D& data) const
{
    if (data.getElementCount() >= elementLimit_) return true;

    for (const auto& [id, element] : data.getElements())
    {
        auto triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (triangle == nullptr || !isTriangleAcceptable(*triangle)) return false;
    }
    return true;
}

bool Shewchuk2DQualityController::isTriangleAcceptable(const TriangleElement& element) const
{
    auto ratio = quality_->computeCircumradiusToShortestEdgeRatio(element);
    if (!ratio.has_value()) return false;
    if (ratio.value() > circumradiusToShortestEdgeRatioBound_) return false;

    auto minimumAngle = geometry_->computeMinAngle(element);
    return !(minimumAngle < minimumAngleThreshold_);
}

double Shewchuk2DQualityController::getTargetElementQuality() const
{
    return circumradiusToShortestEdgeRatioBound_;
}

std::size_t Shewchuk2DQualityController::getElementLimit() const
{
    return elementLimit_;
}

} // namespace Meshing
