#pragma once
#include "Element.h"
#include <array>

namespace Meshing
{

class TetrahedralElement : public Element
{
public:
    TetrahedralElement(size_t id, const std::array<size_t, 4>& nodeIds);

    ElementType getType() const override { return ElementType::TETRAHEDRON; }
    size_t getNodeCount() const override { return 4; }
    const std::vector<size_t>& getNodeIds() const override;
    size_t getId() const override { return id_; }

    double computeVolume() const override;
    double computeQuality() const override;

    // Tet-specific methods
    std::array<size_t, 3> getFace(size_t faceIndex) const;
    std::array<size_t, 2> getEdge(size_t edgeIndex) const;

    // Utility methods
    static constexpr size_t getFaceCount() { return 4; }
    static constexpr size_t getEdgeCount() { return 6; }

private:
    size_t id_;
    std::array<size_t, 4> nodeIds_; // Ordered connectivity
};

} // namespace Meshing