#pragma once

#include "Meshing/Interfaces/IQualityController2D.h"
#include <cstddef>

namespace Meshing
{
class MeshData2D;
class Computer2D;

class Shewchuk2DQualityController : public IQualityController2D
{
public:
    Shewchuk2DQualityController(const Computer2D& computer,
                                double circumradiusToShortestEdgeRatioBound,
                                double minAngleThresholdRadians,
                                std::size_t elementLimit);

    bool isMeshAcceptable(const MeshData2D& data, const MeshConnectivity& connectivity) const override;
    bool isTriangleAcceptable(const TriangleElement& element) const override;
    double getTargetElementQuality() const override;
    std::size_t getElementLimit() const override;

private:
    const Computer2D& computer_;
    double circumradiusToShortestEdgeRatioBound_;
    double minAngleThreshold_;
    std::size_t elementLimit_;
};

} // namespace Meshing
