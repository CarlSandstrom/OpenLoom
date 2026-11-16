#include "ShewchukQualityController2D.h"

#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/Base/TriangleElement.h"

namespace Meshing
{

ShewchukQualityController2D::ShewchukQualityController2D(const MeshData2D& meshData,
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

bool ShewchukQualityController2D::isMeshAcceptable(const MeshData2D& data) const
{
    if (data.getElementCount() >= elementLimit_) return true;

    for (const auto& [id, element] : data.getElements())
    {
        auto triangle = dynamic_cast<const TriangleElement*>(element.get());
        if (triangle == nullptr || !isTriangleAcceptable(*triangle)) return false;
    }
    return true;
}

bool ShewchukQualityController2D::isTriangleAcceptable(const TriangleElement& element) const
{
    auto ratio = quality_->computeCircumradiusToShortestEdgeRatio(element);
    if (!ratio.has_value()) return false;
    if (ratio.value() > circumradiusToShortestEdgeRatioBound_) return false;

    auto minimumAngle = geometry_->computeMinAngle(element);
    return !(minimumAngle < minimumAngleThreshold_);
}

double ShewchukQualityController2D::getTargetElementQuality() const
{
    return circumradiusToShortestEdgeRatioBound_;
}

std::size_t ShewchukQualityController2D::getElementLimit() const
{
    return elementLimit_;
}

} // namespace Meshing
