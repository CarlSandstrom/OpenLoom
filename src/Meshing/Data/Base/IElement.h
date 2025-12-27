#pragma once
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
    virtual const std::vector<size_t>& getNodeIds() const = 0;
    virtual bool getHasNode(size_t nodeId) const = 0;

    // For transaction support
    virtual std::unique_ptr<IElement> clone() const = 0;
};

} // namespace Meshing