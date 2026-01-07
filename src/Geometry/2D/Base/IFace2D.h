#pragma once

#include "Common/Types.h"
#include "IEdgeLoop2D.h"
#include <cstddef>

namespace Geometry2D
{

enum class PointLocation
{
    Inside,
    Outside,
    OnBoundary
};

class IFace2D
{
public:
    virtual ~IFace2D() = default;

    virtual const IEdgeLoop2D& getOuterEdgeLoop() const = 0;
    virtual size_t getHoleCount() const = 0;
    virtual const IEdgeLoop2D& getHoleEdgeLoop(size_t index) const = 0;
    virtual bool hasHoles() const = 0;

    virtual PointLocation classify(const Meshing::Point2D& point) const = 0;
    virtual PointLocation classify(double u, double v) const = 0;
};

} // namespace Geometry2D
