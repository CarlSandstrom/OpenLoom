#pragma once

#include "ElementGeometry2D.h"
#include "ElementQuality2D.h"
#include "Meshing/Interfaces/IQualityController2D.h"
#include <cstddef>
#include <memory>

namespace Meshing
{
class MeshData2D;

class Shewchuk2DQualityController : public IQualityController2D
{
public:
    Shewchuk2DQualityController(const MeshData2D& meshData,
                                double circumradiusToShortestEdgeRatioBound,
                                double minAngleThresholdRadians,
                                std::size_t elementLimit);

    bool isMeshAcceptable(const MeshData2D& data, const MeshConnectivity& connectivity) const override;
    bool isTriangleAcceptable(const TriangleElement& element) const override;
    double getTargetElementQuality() const override;
    std::size_t getElementLimit() const override;

private:
    std::unique_ptr<ElementGeometry2D> geometry_;
    std::unique_ptr<ElementQuality2D> quality_;
    double circumradiusToShortestEdgeRatioBound_;
    double minAngleThreshold_;
    std::size_t elementLimit_;
};

} // namespace Meshing
