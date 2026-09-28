#include "../Base/MeshConnectivity.h"
#include "../3D/TetrahedralElement.h"
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
                auto [it, inserted] =
                    faceToElements_.emplace(key, std::make_pair(elementId, INVALID_ID));
                if (!inserted)
                    it->second.second = elementId;
            }
        }
    }
}

void MeshConnectivity::addElementToConnectivity(size_t elementId)
{
    const IElement* element = geometry_.getElement(elementId);
    if (!element) return;

    const auto& nodeIds = element->getNodeIds();

    // Update node-to-element connectivity
    for (size_t nodeId : nodeIds)
    {
        nodeToElements_[nodeId].push_back(elementId);
    }
}

} // namespace Meshing