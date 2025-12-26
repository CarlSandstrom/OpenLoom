#pragma once
#include "IElement.h"
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
    bool getHasNode(size_t nodeId) const override;

    std::unique_ptr<IElement> clone() const override;

    // Tet-specific methods
    std::array<size_t, 3> getFace(size_t faceIndex) const;
    std::array<size_t, 2> getEdge(size_t edgeIndex) const;
    std::array<std::array<size_t, 3>, 4> getFaces() const;
    bool hasNode(size_t nodeId) const;
    bool containsAll(const std::array<size_t, 3>& nodes) const;

    // Utility methods
    static constexpr size_t getFaceCount() { return 4; }
    static constexpr size_t getEdgeCount() { return 6; }

private:
    std::array<size_t, 4> nodeIds_; // Ordered connectivity
};

} // namespace Meshing