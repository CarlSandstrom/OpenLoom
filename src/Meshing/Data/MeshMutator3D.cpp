#include "MeshMutator3D.h"
#include "MeshConnectivity.h"
#include "Node3D.h"
#include <stdexcept>

namespace Meshing
{

MeshMutator3D::MeshMutator3D(MeshData& geometry) :
    geometry_(geometry)
{
}

void MeshMutator3D::setConnectivity(MeshConnectivity* connectivity)
{
    connectivity_ = connectivity;
}

size_t MeshMutator3D::addNode(const Point3D& coordinates)
{
    size_t id = nextNodeId_++;

    auto node = std::make_unique<Node3D>(coordinates);
    geometry_.addNodeInternal(id, std::move(node));

    // Notify transaction listener
    if (transactionListener_)
    {
        transactionListener_->onNodeAdded(id);
    }

    return id;
}

void MeshMutator3D::moveNode(size_t id, const Point3D& newCoords)
{
    Node3D* node = geometry_.getNodeMutable(id);
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

void MeshMutator3D::removeNode(size_t id)
{
    const Node3D* node = geometry_.getNode(id);
    if (!node)
    {
        throw std::runtime_error("Node ID " + std::to_string(id) + " does not exist");
    }

    // Validate that node can be removed (this would need connectivity info)
    validateNodeRemoval(id);

    // Save node data if transaction is active
    if (transactionListener_)
    {
        Point3D coords = node->getCoordinates();
        transactionListener_->onNodeRemoved(id, coords);
    }

    // Remove node
    geometry_.removeNodeInternal(id);
}

size_t MeshMutator3D::addElement(std::unique_ptr<IElement> element)
{
    size_t id = nextElementId_++;

    geometry_.addElementInternal(id, std::move(element));

    // Notify listener if present
    if (transactionListener_)
    {
        transactionListener_->onElementAdded(id);
    }

    return id;
}

void MeshMutator3D::removeElement(size_t id)
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
    geometry_.removeElementInternal(id);
}

void MeshMutator3D::setTransactionListener(ITransactionListener* listener)
{
    transactionListener_ = listener;
}

void MeshMutator3D::clearTransactionListener()
{
    transactionListener_ = nullptr;
}

void MeshMutator3D::restoreElement(size_t id, std::unique_ptr<IElement> element)
{
    geometry_.addElementInternal(id, std::move(element));

    // Ensure nextElementId_ accounts for restored elements
    if (id >= nextElementId_)
    {
        nextElementId_ = id + 1;
    }
}

void MeshMutator3D::restoreNode(size_t id, const Point3D& coordinates)
{
    Node3D* node = geometry_.getNodeMutable(id);
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

void MeshMutator3D::validateNodeRemoval(size_t nodeId) const
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