#pragma once
#include "IElement.h"
#include <array>

namespace Meshing
{

class TriangleElement : public IElement
{
public:
    TriangleElement(const std::array<size_t, 3>& nodeIds);

    ElementType getType() const override { return ElementType::TRIANGLE; }
    size_t getNodeCount() const override { return 3; }
    const std::vector<size_t>& getNodeIds() const override;
    const std::array<size_t, 3>& getNodeIdArray() const { return nodeIds_; }
    std::array<size_t, 3> getSortedNodeIds() const;

    std::unique_ptr<IElement> clone() const override;

    std::array<size_t, 2> getEdge(size_t edgeIndex) const;

    static constexpr size_t getEdgeCount() { return 3; }

private:
    std::array<size_t, 3> nodeIds_;
};

} // namespace Meshing
