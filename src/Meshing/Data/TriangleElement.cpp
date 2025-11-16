#include "TriangleElement.h"
#include <algorithm>

namespace Meshing
{

TriangleElement::TriangleElement(const std::array<size_t, 3>& nodeIds) :
    nodeIds_(nodeIds)
{
}

const std::vector<size_t>& TriangleElement::getNodeIds() const
{
    static thread_local std::vector<size_t> nodeVector;
    nodeVector.assign(nodeIds_.begin(), nodeIds_.end());
    return nodeVector;
}

std::array<size_t, 2> TriangleElement::getEdge(size_t edgeIndex) const
{
    switch (edgeIndex)
    {
    case 0:
        return {nodeIds_[0], nodeIds_[1]};
    case 1:
        return {nodeIds_[1], nodeIds_[2]};
    case 2:
        return {nodeIds_[2], nodeIds_[0]};
    default:
        return {nodeIds_[0], nodeIds_[1]};
    }
}

std::array<size_t, 3> TriangleElement::getSortedNodeIds() const
{
    auto sorted = nodeIds_;
    std::sort(sorted.begin(), sorted.end());
    return sorted;
}

std::unique_ptr<IElement> TriangleElement::clone() const
{
    return std::make_unique<TriangleElement>(nodeIds_);
}

} // namespace Meshing
