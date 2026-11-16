#pragma once

#include "Meshing/Core/2D/General/ElementGeometry2D.h"
#include "Meshing/Core/2D/General/ElementQuality2D.h"
#include "Meshing/Interfaces/2D/IQualityController2D.h"
#include <cstddef>
#include <memory>

namespace Meshing
{
class MeshData2D;

class ShewchukQualityController2D : public IQualityController2D
{
public:
    ShewchukQualityController2D(const MeshData2D& meshData,
                                double circumradiusToShortestEdgeRatioBound,
                                double minimumAngleThresholdRadians,
                                std::size_t elementLimit);

    bool isMeshAcceptable(const MeshData2D& data) const override;
    bool isTriangleAcceptable(const TriangleElement& element) const override;
    double getTargetElementQuality() const override;
    std::size_t getElementLimit() const override;

private:
    std::unique_ptr<ElementGeometry2D> geometry_;
    std::unique_ptr<ElementQuality2D> quality_;
    double circumradiusToShortestEdgeRatioBound_;
    double minimumAngleThreshold_;
    std::size_t elementLimit_;
};

} // namespace Meshing
