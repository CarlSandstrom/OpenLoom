#include "TetrahedralElement.h"
#include <algorithm>
#include <cmath>

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

double TetrahedralElement::computeVolume() const
{
    // Note: This method requires access to actual node coordinates
    // In a complete mesh system, this would be computed as:
    // V = |det(v1-v0, v2-v0, v3-v0)| / 6
    // where v0, v1, v2, v3 are the coordinates of the four nodes

    // For now, returning zero since we don't have access to
    // the mesh data structure containing node coordinates
    return 0.0; // Requires mesh context for coordinate access
}

double TetrahedralElement::computeQuality() const
{
    // Quality metric for tetrahedron using radius ratio:
    // Quality = (72 * sqrt(3) * V) / (sum of squared edge lengths)^(3/2)
    // where V is volume and edge lengths are computed from node coordinates

    // Alternative quality metrics include:
    // - Aspect ratio: ratio of longest to shortest edge
    // - Condition number: measures element distortion
    // - Shape measure: compares to ideal tetrahedron

    // For now, returning zero since we don't have access to
    // the mesh data structure containing node coordinates
    return 0.0; // Requires mesh context for coordinate access
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

std::unique_ptr<Element> TetrahedralElement::clone() const
{
    return std::make_unique<TetrahedralElement>(nodeIds_);
}

} // namespace Meshing