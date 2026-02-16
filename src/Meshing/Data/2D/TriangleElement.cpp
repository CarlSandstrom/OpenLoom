#include "TriangleElement.h"
#include "Common/Exceptions/MeshException.h"
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

bool TriangleElement::hasNode(size_t nodeId) const
{
    return nodeIds_[0] == nodeId || nodeIds_[1] == nodeId || nodeIds_[2] == nodeId;
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
        CMESH_THROW_CODE(cMesh::MeshException,
                         cMesh::MeshException::ErrorCode::INVALID_OPERATION,
                         "Invalid edge index " + std::to_string(edgeIndex) + " for triangle (valid: 0-2)");
    }
}

size_t TriangleElement::getOppositeNode(size_t edgeNode1, size_t edgeNode2) const
{
    for (size_t n : nodeIds_)
    {
        if (n != edgeNode1 && n != edgeNode2)
        {
            return n;
        }
    }
    CMESH_THROW_CODE(cMesh::MeshException,
                     cMesh::MeshException::ErrorCode::INVALID_OPERATION,
                     "No opposite node found for edge (" + std::to_string(edgeNode1) +
                         ", " + std::to_string(edgeNode2) + ") in triangle");
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
