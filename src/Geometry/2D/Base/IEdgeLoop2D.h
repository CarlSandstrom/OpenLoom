#pragma once

#include <string>
#include <vector>

namespace Geometry2D
{

class IEdgeLoop2D
{
public:
    virtual ~IEdgeLoop2D() = default;

    virtual std::vector<std::string> getEdgeIds() const = 0;
    virtual bool isClosed() const = 0;
    virtual bool isCounterClockwise() const = 0;
};

} // namespace Geometry2D
