#include "MeshConnectivity.h"
#include "TetrahedralElement.h"
#include <algorithm>

namespace Meshing
{

constexpr size_t INVALID_ID = SIZE_MAX;

MeshConnectivity::MeshConnectivity(const MeshData3D& geometry) :
    geometry_(geometry)
{
    rebuildConnectivity();
}

const std::vector<size_t>& MeshConnectivity::getNodeElements(size_t nodeId) const
{
    auto it = nodeToElements_.find(nodeId);
    if (it != nodeToElements_.end())
    {
        return it->second;
    }

    // Return empty vector if node not found
    static const std::vector<size_t> emptyVector;
    return emptyVector;
}

const std::pair<size_t, size_t>& MeshConnectivity::getFaceElements(const FaceKey& face) const
{
    auto it = faceToElements_.find(face);
    if (it != faceToElements_.end())
    {
        return it->second;
    }

    // Return a static pair with invalid IDs if face not found
    static const std::pair<size_t, size_t> invalidPair{INVALID_ID, INVALID_ID};
    return invalidPair;
}

void MeshConnectivity::rebuildConnectivity()
{
    nodeToElements_.clear();
    faceToElements_.clear();

    // Initialize empty vectors for all nodes
    for (const auto& [nodeId, node] : geometry_.getNodes())
    {
        nodeToElements_[nodeId] = std::vector<size_t>{};
    }

    // Build connectivity maps
    buildNodeToElementsMap();
    buildFaceToElementsMap();
}

bool MeshConnectivity::canRemoveNode(size_t nodeId) const
{
    auto it = nodeToElements_.find(nodeId);
    return (it == nodeToElements_.end()) || it->second.empty();
}

void MeshConnectivity::buildNodeToElementsMap()
{
    for (const auto& [elementId, element] : geometry_.getElements())
    {
        addElementToConnectivity(elementId);
    }
}

void MeshConnectivity::buildFaceToElementsMap()
{
    for (const auto& [elementId, element] : geometry_.getElements())
    {
        if (element->getType() == ElementType::TETRAHEDRON)
        {
            const auto* tet = static_cast<const TetrahedralElement*>(element.get());

            // A tetrahedron has 4 faces
            for (size_t i = 0; i < 4; ++i)
            {
                std::array<size_t, 3> faceNodes = tet->getFace(i);

                // Create canonical face key (automatically sorted)
                FaceKey key = makeFaceKey(faceNodes);

                // Add this element to the face
                auto& elemPair = faceToElements_[key];
                if (elemPair.first == INVALID_ID)
                {
                    elemPair.first = elementId; // First element on this face
                }
                else
                {
                    elemPair.second = elementId; // Second element on this face
                }
            }
        }
    }
}

void MeshConnectivity::addElementToConnectivity(size_t elementId)
{
    const IElement* elem = geometry_.getElement(elementId);
    if (!elem) return;

    const auto& nodeIds = elem->getNodeIds();

    // Update node-to-element connectivity
    for (size_t nodeId : nodeIds)
    {
        nodeToElements_[nodeId].push_back(elementId);
    }
}

void MeshConnectivity::removeElementFromConnectivity(size_t elementId)
{
    const IElement* elem = geometry_.getElement(elementId);
    if (!elem) return;

    const auto& nodeIds = elem->getNodeIds();

    // Remove from node-to-element connectivity
    for (size_t nodeId : nodeIds)
    {
        auto& elemList = nodeToElements_[nodeId];
        elemList.erase(std::remove(elemList.begin(), elemList.end(), elementId),
                       elemList.end());
    }

    // Remove from face-to-element connectivity
    if (elem->getType() == ElementType::TETRAHEDRON)
    {
        const auto* tet = static_cast<const TetrahedralElement*>(elem);

        for (size_t i = 0; i < 4; ++i)
        {
            std::array<size_t, 3> faceNodes = tet->getFace(i);
            FaceKey key = makeFaceKey(faceNodes);

            auto& elemPair = faceToElements_[key];
            if (elemPair.first == elementId)
            {
                // Shift second to first position
                elemPair.first = elemPair.second;
                elemPair.second = INVALID_ID;
            }
            else if (elemPair.second == elementId)
            {
                elemPair.second = INVALID_ID;
            }

            // If face has no elements, remove it from map
            if (elemPair.first == INVALID_ID && elemPair.second == INVALID_ID)
            {
                faceToElements_.erase(key);
            }
        }
    }
}

} // namespace Meshing