#include "MeshData.h"
#include "TetrahedralElement.h"
#include <algorithm>
#include <stdexcept>

namespace Meshing
{

constexpr size_t INVALID_ID = SIZE_MAX;

MeshData::MeshData()
{
}

size_t MeshData::addNode(const std::array<double, 3>& coordinates)
{
    size_t id = nodes_.size();

    auto node = std::make_unique<Node>(id, coordinates);
    nodes_.push_back(std::move(node));

    // Initialize empty element list for this node
    nodeToElements_[id] = std::vector<size_t>{};

    // Notify transaction listener
    if (transactionListener_)
    {
        transactionListener_->onNodeAdded(id);
    }

    return id;
}

const std::pair<size_t, size_t>& MeshData::getFaceElements(const FaceKey& face) const
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

const std::vector<size_t>& MeshData::getNodeElements(size_t nodeId) const
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

void MeshData::setTransactionListener(ITransactionListener* listener)
{
    transactionListener_ = listener;
}

void MeshData::clearTransactionListener()
{
    transactionListener_ = nullptr;
}

size_t MeshData::addElement(std::unique_ptr<Element> element)
{
    size_t id = element->getId();
    elements_.push_back(std::move(element));
    addElementToConnectivity_(id);

    // Notify listener if present
    if (transactionListener_)
    {
        transactionListener_->onElementAdded(id);
    }

    return id;
}

void MeshData::removeElement(size_t id)
{
    // Clone element before removing (if listener needs it)
    std::unique_ptr<Element> clone;
    if (transactionListener_)
    {
        clone = elements_[id]->clone();
    }

    // Remove from connectivity
    removeElementFromConnectivity_(id);

    // Notify listener BEFORE actually removing
    if (transactionListener_)
    {
        transactionListener_->onElementRemoved(id, std::move(clone));
    }

    // Actually remove
    elements_[id].reset();
}

void MeshData::moveNode(size_t id, const std::array<double, 3>& newCoords)
{
    if (id >= nodes_.size() || !nodes_[id])
    {
        throw std::runtime_error("Node ID " + std::to_string(id) + " does not exist");
    }

    Node* node = nodes_[id].get();

    // Notify transaction listener BEFORE modification
    if (transactionListener_)
    {
        auto oldCoords = node->getCoordinates();
        transactionListener_->onNodeModified(id, oldCoords);
    }

    // Perform modification
    node->setCoordinates(newCoords);
}

void MeshData::removeNode(size_t id)
{
    if (id >= nodes_.size() || !nodes_[id])
    {
        throw std::runtime_error("Node ID " + std::to_string(id) + " does not exist");
    }

    // Check if node is still referenced by any elements
    auto it = nodeToElements_.find(id);
    if (it != nodeToElements_.end() && !it->second.empty())
    {
        throw std::runtime_error(
            "Cannot remove node " + std::to_string(id) +
            ": still referenced by " + std::to_string(it->second.size()) + " element(s)");
    }

    // Save node data if transaction is active
    if (transactionListener_)
    {
        auto coords = nodes_[id]->getCoordinates();
        transactionListener_->onNodeRemoved(id, coords);
    }

    // Remove from connectivity map
    nodeToElements_.erase(id);

    // Remove node
    nodes_[id].reset();
}

void MeshData::restoreElement(size_t id, std::unique_ptr<Element> element)
{
    // Public method for transaction to restore elements
    elements_[id] = std::move(element);
    addElementToConnectivity_(id);
}

void MeshData::restoreNode(size_t id, const std::array<double, 3>& coordinates)
{
    nodes_[id]->setCoordinates(coordinates);
}

void MeshData::addElementToConnectivity_(size_t elementId)
{
    const Element* elem = elements_[elementId].get();
    const auto& nodeIds = elem->getNodeIds();

    // Update node-to-element connectivity
    for (size_t nodeId : nodeIds)
    {
        nodeToElements_[nodeId].push_back(elementId);
    }

    // Update face-to-element connectivity
    if (elem->getType() == ElementType::TETRAHEDRON)
    {
        const auto* tet = static_cast<const TetrahedralElement*>(elem);

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

void MeshData::removeElementFromConnectivity_(size_t elementId)
{
    const Element* elem = elements_[elementId].get();
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