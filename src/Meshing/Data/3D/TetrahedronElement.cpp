#include "Meshing/Data/3D/TetrahedronElement.h"
#include "Common/Exceptions/MeshException.h"

#include <algorithm>
#include <string>

namespace Meshing
{

TetrahedronElement::TetrahedronElement(const std::array<NodeId, 4>& nodeIds) :
    nodeIds_(nodeIds),
    nodeIdsVector_(nodeIds_.begin(), nodeIds_.end())
{
}

const std::vector<NodeId>& TetrahedronElement::getNodeIds() const
{
    return nodeIdsVector_;
}

bool TetrahedronElement::hasNode(NodeId nodeId) const
{
    return std::find(nodeIds_.begin(), nodeIds_.end(), nodeId) != nodeIds_.end();
}

std::array<NodeId, 3> TetrahedronElement::getFace(size_t faceIndex) const
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
        OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                            OpenLoom::MeshException::ErrorCode::INVALID_OPERATION,
                            "TetrahedronElement::getFace: face index " + std::to_string(faceIndex) +
                                " is out of range 0-3");
    }
}

std::array<std::array<NodeId, 3>, 4> TetrahedronElement::getFaces() const
{
    return {{{nodeIds_[1], nodeIds_[2], nodeIds_[3]},
             {nodeIds_[0], nodeIds_[3], nodeIds_[2]},
             {nodeIds_[0], nodeIds_[1], nodeIds_[3]},
             {nodeIds_[0], nodeIds_[2], nodeIds_[1]}}};
}

} // namespace Meshing