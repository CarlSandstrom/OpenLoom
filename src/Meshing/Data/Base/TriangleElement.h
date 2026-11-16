#pragma once

#include "Meshing/Data/Base/IElement.h"

#include <array>
#include <vector>

namespace Meshing
{

class TriangleElement : public IElement
{
public:
    explicit TriangleElement(const std::array<NodeId, 3>& nodeIds);

    ElementType getType() const override { return ElementType::TRIANGLE; }
    size_t getNodeCount() const override { return 3; }
    const std::vector<NodeId>& getNodeIds() const override;
    bool hasNode(NodeId nodeId) const override;

    const std::array<NodeId, 3>& getNodeIdArray() const { return nodeIds_; }

    std::array<NodeId, 2> getEdge(size_t edgeIndex) const;

    /// Returns the node ID that is not part of the given edge.
    /// Throws if neither edgeNode1 nor edgeNode2 are found in this triangle.
    NodeId getOppositeNode(NodeId edgeNode1, NodeId edgeNode2) const;

private:
    std::array<NodeId, 3> nodeIds_;
    std::vector<NodeId> nodeIdsVector_; // Cached vector for IElement interface
};

} // namespace Meshing
