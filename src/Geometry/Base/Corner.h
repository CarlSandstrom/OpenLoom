#pragma once

#include <array>
#include <string>

namespace Geometry
{

/**
 * @brief Abstract interface for geometric vertices/corners
 */
class Corner
{
public:
    virtual ~Corner() = default;

    virtual std::array<double, 3> getPoint() const = 0;
    virtual std::string getId() const = 0;
};

} // namespace Geometry