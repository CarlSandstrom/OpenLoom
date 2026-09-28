#include "TetrahedralElement.h"
#include <algorithm>

namespace Meshing
{

TetrahedralElement::TetrahedralElement(const std::array<size_t, 4>& nodeIds) :
    nodeIds_(nodeIds),
    nodeIdsVector_(nodeIds_.begin(), nodeIds_.end())
{
}

const std::vector<size_t>& TetrahedralElement::getNodeIds() const
{
    return nodeIdsVector_;
}

bool TetrahedralElement::hasNode(size_t nodeId) const
{
    return nodeIds_[0] == nodeId || nodeIds_[1] == nodeId || nodeIds_[2] == nodeId || nodeIds_[3] == nodeId;
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

std::array<std::array<size_t, 3>, 4> TetrahedralElement::getFaces() const
{
    return {{{nodeIds_[1], nodeIds_[2], nodeIds_[3]},
             {nodeIds_[0], nodeIds_[3], nodeIds_[2]},
             {nodeIds_[0], nodeIds_[1], nodeIds_[3]},
             {nodeIds_[0], nodeIds_[2], nodeIds_[1]}}};
}

std::unique_ptr<IElement> TetrahedralElement::clone() const
{
    return std::make_unique<TetrahedralElement>(nodeIds_);
}

} // namespace Meshing