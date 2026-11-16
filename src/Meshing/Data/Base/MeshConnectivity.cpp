#include "../Base/MeshConnectivity.h"
#include "../3D/TetrahedronElement.h"
#include <algorithm>

namespace Meshing
{

MeshConnectivity::MeshConnectivity(const MeshData3D& geometry) :
    geometry_(geometry)
{
    rebuildConnectivity();
}

const std::vector<ElementId>& MeshConnectivity::getNodeElements(NodeId nodeId) const
{
    auto it = nodeToElements_.find(nodeId);
    if (it != nodeToElements_.end())
    {
        return it->second;
    }

    return {};
}

const std::pair<ElementId, ElementId>& MeshConnectivity::getFaceElements(const FaceKey& face) const
{
    auto it = faceToElements_.find(face);
    if (it != faceToElements_.end())
    {
        return it->second;
    }

    // Return a static pair with invalid IDs if face not found
    static const std::pair<ElementId, ElementId> invalidPair{ElementId::INVALID, ElementId::INVALID};
    return invalidPair;
}

void MeshConnectivity::rebuildConnectivity()
{
    nodeToElements_.clear();
    faceToElements_.clear();

    // Initialize empty vectors for all nodes
    for (const auto& [nodeId, node] : geometry_.getNodes())
    {
        nodeToElements_[nodeId] = std::vector<ElementId>{};
    }

    for (const auto& [elementId, element] : geometry_.getElements())
    {
        addElement(elementId);
    }
}

bool MeshConnectivity::canRemoveNode(NodeId nodeId) const
{
    auto it = nodeToElements_.find(nodeId);
    return (it == nodeToElements_.end()) || it->second.empty();
}

void MeshConnectivity::addElement(ElementId elementId)
{
    auto element = geometry_.getElement(elementId);
    if (!element) return;

    for (auto nodeId : element->getNodeIds())
    {
        nodeToElements_[nodeId].push_back(elementId);
    }

    if (element->getType() != ElementType::TETRAHEDRON) return;

    auto tetrahedron = static_cast<const TetrahedronElement*>(element);
    for (size_t i = 0; i < 4; ++i)
    {
        auto [entry, inserted] =
            faceToElements_.emplace(makeFaceKey(tetrahedron->getFace(i)), std::make_pair(elementId, ElementId::INVALID));
        if (!inserted) entry->second.second = elementId;
    }
}

void MeshConnectivity::removeElement(ElementId elementId)
{
    auto element = geometry_.getElement(elementId);
    if (!element) return;

    for (auto nodeId : element->getNodeIds())
    {
        auto& elementIds = nodeToElements_[nodeId];
        elementIds.erase(std::remove(elementIds.begin(), elementIds.end(), elementId), elementIds.end());
    }

    if (element->getType() != ElementType::TETRAHEDRON) return;

    auto tetrahedron = static_cast<const TetrahedronElement*>(element);
    for (size_t i = 0; i < 4; ++i)
    {
        auto entry = faceToElements_.find(makeFaceKey(tetrahedron->getFace(i)));
        if (entry == faceToElements_.end()) continue;

        auto& [first, second] = entry->second;
        if (first == elementId) first = second;
        second = ElementId::INVALID;
        if (first == ElementId::INVALID) faceToElements_.erase(entry);
    }
}

} // namespace Meshing
