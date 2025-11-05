#pragma once

#include <cstddef>

namespace Meshing
{
class MeshData;
class MeshConnectivity;

// Interface responsible for evaluating mesh quality and optionally
// providing refinement decisions or thresholds.
class IQualityController
{
public:
    virtual ~IQualityController() = default;

    // Called after generation or refinement to assess overall quality.
    // Return true if acceptable, false if further refinement needed.
    virtual bool isMeshAcceptable(const MeshData& data, const MeshConnectivity& connectivity) const = 0;

    // Optional target minimum quality metric for elements.
    virtual double getTargetElementQuality() const = 0;

    // Maximum allowed number of elements before refinement stops (to avoid runaway growth).
    virtual std::size_t getElementLimit() const = 0;
};

} // namespace Meshing
