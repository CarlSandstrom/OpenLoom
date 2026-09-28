#pragma once
#include "../Base/IElement.h"
#include <array>

namespace Meshing
{

class TetrahedralElement : public IElement
{
public:
    explicit TetrahedralElement(const std::array<size_t, 4>& nodeIds);

    ElementType getType() const override { return ElementType::TETRAHEDRON; }
    size_t getNodeCount() const override { return 4; }
    const std::vector<size_t>& getNodeIds() const override;
    bool hasNode(size_t nodeId) const override;

    // Tet-specific methods
    std::array<size_t, 3> getFace(size_t faceIndex) const;
    std::array<std::array<size_t, 3>, 4> getFaces() const;

private:
    std::array<size_t, 4> nodeIds_; // Ordered connectivity
    std::vector<size_t> nodeIdsVector_;  // Cached vector for IElement interface
};

} // namespace Meshing