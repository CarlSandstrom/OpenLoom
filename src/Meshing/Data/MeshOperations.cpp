#include "MeshOperations.h"
#include "MeshConnectivity.h"
#include "Node.h"
#include <stdexcept>

namespace Meshing
{

MeshOperations::MeshOperations(MeshData& geometry) :
    geometry_(geometry)
{
}

void MeshOperations::setConnectivity(MeshConnectivity* connectivity)
{
    connectivity_ = connectivity;
}

size_t MeshOperations::addNode(const Point3D& coordinates)
{
    size_t id = nextNodeId_++;

    auto node = std::make_unique<Node>(coordinates);
    geometry_.addNodeInternal_(id, std::move(node));

    // Notify transaction listener
    if (transactionListener_)
    {
        transactionListener_->onNodeAdded(id);
    }

    return id;
}

void MeshOperations::moveNode(size_t id, const Point3D& newCoords)
{
    Node* node = geometry_.getNodeMutable_(id);
    if (!node)
    {
        throw std::runtime_error("Node ID " + std::to_string(id) + " does not exist");
    }

    // Notify transaction listener BEFORE modification
    if (transactionListener_)
    {
        Point3D oldCoords = node->getCoordinates();
        transactionListener_->onNodeModified(id, oldCoords);
    }

    // Perform modification
    node->setCoordinates(newCoords);
}

void MeshOperations::removeNode(size_t id)
{
    const Node* node = geometry_.getNode(id);
    if (!node)
    {
        throw std::runtime_error("Node ID " + std::to_string(id) + " does not exist");
    }

    // Validate that node can be removed (this would need connectivity info)
    validateNodeRemoval_(id);

    // Save node data if transaction is active
    if (transactionListener_)
    {
        Point3D coords = node->getCoordinates();
        transactionListener_->onNodeRemoved(id, coords);
    }

    // Remove node
    geometry_.removeNodeInternal_(id);
}

size_t MeshOperations::addElement(std::unique_ptr<IElement> element)
{
    size_t id = nextElementId_++;

    geometry_.addElementInternal_(id, std::move(element));

    // Notify listener if present
    if (transactionListener_)
    {
        transactionListener_->onElementAdded(id);
    }

    return id;
}

void MeshOperations::removeElement(size_t id)
{
    const IElement* element = geometry_.getElement(id);
    if (!element)
    {
        throw std::runtime_error("Element ID " + std::to_string(id) + " does not exist");
    }

    // Clone element before removing (if listener needs it)
    std::unique_ptr<IElement> clone;
    if (transactionListener_)
    {
        clone = element->clone();
    }

    // Notify listener BEFORE actually removing
    if (transactionListener_)
    {
        transactionListener_->onElementRemoved(id, std::move(clone));
    }

    // Actually remove
    geometry_.removeElementInternal_(id);
}

void MeshOperations::setTransactionListener(ITransactionListener* listener)
{
    transactionListener_ = listener;
}

void MeshOperations::clearTransactionListener()
{
    transactionListener_ = nullptr;
}

void MeshOperations::restoreElement(size_t id, std::unique_ptr<IElement> element)
{
    geometry_.addElementInternal_(id, std::move(element));

    // Ensure nextElementId_ accounts for restored elements
    if (id >= nextElementId_)
    {
        nextElementId_ = id + 1;
    }
}

void MeshOperations::restoreNode(size_t id, const Point3D& coordinates)
{
    Node* node = geometry_.getNodeMutable_(id);
    if (node)
    {
        node->setCoordinates(coordinates);
    }

    // Ensure nextNodeId_ accounts for restored nodes
    if (id >= nextNodeId_)
    {
        nextNodeId_ = id + 1;
    }
}

void MeshOperations::validateNodeRemoval_(size_t nodeId) const
{
    if (connectivity_ && !connectivity_->canRemoveNode(nodeId))
    {
        const auto& elements = connectivity_->getNodeElements(nodeId);
        throw std::runtime_error(
            "Cannot remove node " + std::to_string(nodeId) +
            ": still referenced by " + std::to_string(elements.size()) + " element(s)");
    }
}

} // namespace Meshing