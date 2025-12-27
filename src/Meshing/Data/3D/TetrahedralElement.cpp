#include "TetrahedralElement.h"
#include <algorithm>

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

bool TetrahedralElement::getHasNode(size_t nodeId) const
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

std::array<std::array<size_t, 3>, 4> TetrahedralElement::getFaces() const
{
    return {{{nodeIds_[1], nodeIds_[2], nodeIds_[3]},
             {nodeIds_[0], nodeIds_[3], nodeIds_[2]},
             {nodeIds_[0], nodeIds_[1], nodeIds_[3]},
             {nodeIds_[0], nodeIds_[2], nodeIds_[1]}}};
}

bool TetrahedralElement::hasNode(size_t nodeId) const
{
    return std::find(nodeIds_.begin(), nodeIds_.end(), nodeId) != nodeIds_.end();
}

bool TetrahedralElement::containsAll(const std::array<size_t, 3>& nodes) const
{
    for (size_t id : nodes)
    {
        if (!hasNode(id))
        {
            return false;
        }
    }
    return true;
}

std::unique_ptr<IElement> TetrahedralElement::clone() const
{
    return std::make_unique<TetrahedralElement>(nodeIds_);
}

} // namespace Meshing