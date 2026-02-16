#pragma once

#include "Common/Types.h"
namespace Meshing
{
struct Circle2D
{
    Point2D center;
    double radius = 0.0;
};

enum class EdgeRole
{
    BOUNDARY,
    INTERIOR
};

struct ConstrainedSegment2D
{
    size_t nodeId1;
    size_t nodeId2;
    EdgeRole role = EdgeRole::BOUNDARY;
};

} // namespace Meshing