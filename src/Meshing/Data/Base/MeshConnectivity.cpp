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

    // Build connectivity maps
    buildNodeToElementsMap();
    buildFaceToElementsMap();
}

bool MeshConnectivity::canRemoveNode(NodeId nodeId) const
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
            const auto* tet = static_cast<const TetrahedronElement*>(element.get());

            // A tetrahedron has 4 faces
            for (size_t i = 0; i < 4; ++i)
            {
                std::array<NodeId, 3> faceNodes = tet->getFace(i);

                // Create canonical face key (automatically sorted)
                FaceKey key = makeFaceKey(faceNodes);

                // Add this element to the face
                auto [it, inserted] =
                    faceToElements_.emplace(key, std::make_pair(elementId, ElementId::INVALID));
                if (!inserted)
                    it->second.second = elementId;
            }
        }
    }
}

void MeshConnectivity::addElementToConnectivity(ElementId elementId)
{
    const IElement* element = geometry_.getElement(elementId);
    if (!element) return;

    const auto& nodeIds = element->getNodeIds();

    // Update node-to-element connectivity
    for (NodeId nodeId : nodeIds)
    {
        nodeToElements_[nodeId].push_back(elementId);
    }
}

} // namespace Meshing