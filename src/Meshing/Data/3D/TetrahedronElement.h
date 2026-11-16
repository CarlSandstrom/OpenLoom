#pragma once

#include "Meshing/Data/Base/IElement.h"

#include <array>

namespace Meshing
{

class TetrahedronElement : public IElement
{
public:
    explicit TetrahedronElement(const std::array<NodeId, 4>& nodeIds);

    ElementType getType() const override { return ElementType::TETRAHEDRON; }
    size_t getNodeCount() const override { return 4; }
    const std::vector<NodeId>& getNodeIds() const override;
    bool hasNode(NodeId nodeId) const override;

    // Tetrahedron-specific methods
    std::array<NodeId, 3> getFace(size_t faceIndex) const;
    std::array<std::array<NodeId, 3>, 4> getFaces() const;

private:
    std::array<NodeId, 4> nodeIds_;     // Ordered connectivity
    std::vector<NodeId> nodeIdsVector_; // Cached vector for IElement interface
};

} // namespace Meshing