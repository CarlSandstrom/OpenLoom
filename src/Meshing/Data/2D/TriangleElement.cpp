#include "Meshing/Data/2D/TriangleElement.h"

#include "Common/Exceptions/MeshException.h"

#include <algorithm>
#include <string>

namespace Meshing
{

TriangleElement::TriangleElement(const std::array<size_t, 3>& nodeIds) :
    nodeIds_(nodeIds),
    nodeIdsVector_(nodeIds_.begin(), nodeIds_.end())
{
}

const std::vector<size_t>& TriangleElement::getNodeIds() const
{
    return nodeIdsVector_;
}

bool TriangleElement::hasNode(size_t nodeId) const
{
    return std::find(nodeIds_.begin(), nodeIds_.end(), nodeId) != nodeIds_.end();
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
        OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                            OpenLoom::MeshException::ErrorCode::INVALID_OPERATION,
                            "Invalid edge index " + std::to_string(edgeIndex) + " for triangle (valid: 0-2)");
    }
}

size_t TriangleElement::getOppositeNode(size_t edgeNode1, size_t edgeNode2) const
{
    for (auto nodeId : nodeIds_)
    {
        if (nodeId != edgeNode1 && nodeId != edgeNode2) return nodeId;
    }
    OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                        OpenLoom::MeshException::ErrorCode::INVALID_OPERATION,
                        "No opposite node found for edge (" + std::to_string(edgeNode1) +
                            ", " + std::to_string(edgeNode2) + ") in triangle");
}

} // namespace Meshing
