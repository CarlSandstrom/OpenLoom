#pragma once

#include "Common/Types.h"

#include <cstddef>
#include <memory>
#include <vector>

namespace Meshing
{

enum class ElementType
{
    TETRAHEDRON,
    HEXAHEDRON,
    PRISM,
    PYRAMID,
    TRIANGLE, // For surface mesh
    QUADRILATERAL
};

class IElement
{
public:
    virtual ~IElement() = default;

    virtual ElementType getType() const = 0;
    virtual size_t getNodeCount() const = 0;
    virtual const std::vector<NodeId>& getNodeIds() const = 0;
    virtual bool hasNode(NodeId nodeId) const = 0;
};

} // namespace Meshing