#pragma once

#include "Common/Types.h"
namespace Meshing
{
struct Circle2D
{
    Point2D center;
    double radius;
};

struct ConstrainedSegment2D
{
    size_t nodeId1;
    size_t nodeId2;
};

} // namespace Meshing