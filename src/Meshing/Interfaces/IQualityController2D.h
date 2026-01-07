#pragma once

#include <cstddef>

namespace Meshing
{
class MeshData2D;
class MeshConnectivity;
class TriangleElement;

class IQualityController2D
{
public:
    virtual ~IQualityController2D() = default;

    virtual bool isMeshAcceptable(const MeshData2D& data, const MeshConnectivity& connectivity) const = 0;

    virtual bool isTriangleAcceptable(const TriangleElement& element) const = 0;

    virtual double getTargetElementQuality() const = 0;

    virtual std::size_t getElementLimit() const = 0;
};

} // namespace Meshing
