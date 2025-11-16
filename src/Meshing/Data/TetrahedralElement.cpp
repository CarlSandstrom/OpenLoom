#include "TetrahedralElement.h"

namespace Meshing
{

TetrahedralElement::TetrahedralElement(const std::array<size_t, 4>& nodeIds) :
    nodeIds_(nodeIds)
{
}

const std::vector<size_t>& TetrahedralElement::getNodeIds() const
{
    static thread_local std::vector<size_t> nodeVector;
    nodeVector.assign(nodeIds_.begin(), nodeIds_.end());
    return nodeVector;
}

std::array<size_t, 3> TetrahedralElement::getFace(size_t faceIndex) const
{
    // Standard tetrahedral face ordering (opposite to node)
    switch (faceIndex)
    {
    case 0:
        return {nodeIds_[1], nodeIds_[2], nodeIds_[3]}; // Face opposite to node 0
    case 1:
        return {nodeIds_[0], nodeIds_[3], nodeIds_[2]}; // Face opposite to node 1
    case 2:
        return {nodeIds_[0], nodeIds_[1], nodeIds_[3]}; // Face opposite to node 2
    case 3:
        return {nodeIds_[0], nodeIds_[2], nodeIds_[1]}; // Face opposite to node 3
    default:
        return {nodeIds_[0], nodeIds_[1], nodeIds_[2]}; // Default to first face
    }
}

std::array<size_t, 2> TetrahedralElement::getEdge(size_t edgeIndex) const
{
    // Standard tetrahedral edge ordering
    switch (edgeIndex)
    {
    case 0:
        return {nodeIds_[0], nodeIds_[1]};
    case 1:
        return {nodeIds_[0], nodeIds_[2]};
    case 2:
        return {nodeIds_[0], nodeIds_[3]};
    case 3:
        return {nodeIds_[1], nodeIds_[2]};
    case 4:
        return {nodeIds_[1], nodeIds_[3]};
    case 5:
        return {nodeIds_[2], nodeIds_[3]};
    default:
        return {nodeIds_[0], nodeIds_[1]}; // Default to first edge
    }
}

std::unique_ptr<IElement> TetrahedralElement::clone() const
{
    return std::make_unique<TetrahedralElement>(nodeIds_);
}

} // namespace Meshing