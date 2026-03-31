#include "MeshMutator3D.h"
#include "../Base/MeshConnectivity.h"
#include "Common/Exceptions/MeshException.h"
#include "Node3D.h"

namespace Meshing
{

MeshMutator3D::MeshMutator3D(MeshData3D& geometry) :
    geometry_(geometry)
{
    // Initialize ID counters from existing mesh data to avoid collisions
    for (const auto& [id, node] : geometry_.getNodes())
    {
        if (id >= nextNodeId_)
        {
            nextNodeId_ = id + 1;
        }
    }
    for (const auto& [id, element] : geometry_.getElements())
    {
        if (id >= nextElementId_)
        {
            nextElementId_ = id + 1;
        }
    }
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

size_t MeshMutator3D::addBoundaryNode(const Point3D& coordinates,
                                      const std::vector<std::string>& geometryIds)
{
    size_t id = nextNodeId_++;

    auto node = std::make_unique<Node3D>(coordinates, geometryIds);
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
        throw OpenLoom::MeshEntityNotFoundException("Node", id, std::string(__FILE__) + ":" + std::to_string(__LINE__));
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
        throw OpenLoom::MeshEntityNotFoundException("Node", id, std::string(__FILE__) + ":" + std::to_string(__LINE__));
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
        throw OpenLoom::MeshEntityNotFoundException("Element", id, std::string(__FILE__) + ":" + std::to_string(__LINE__));
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
    else
    {
        // Node was deleted; recreate it
        auto newNode = std::make_unique<Node3D>(coordinates);
        geometry_.addNodeInternal(id, std::move(newNode));
    }

    // Ensure nextNodeId_ accounts for restored nodes
    if (id >= nextNodeId_)
    {
        nextNodeId_ = id + 1;
    }
}

// ========== Curve Segment Operations ==========

void MeshMutator3D::addCurveSegment(const CurveSegment& segment)
{
    geometry_.curveSegmentManager_.addSegment(segment);
}

std::pair<size_t, size_t> MeshMutator3D::splitCurveSegment(size_t segmentId, size_t newNodeId, double tMid)
{
    return geometry_.curveSegmentManager_.splitAt(segmentId, newNodeId, tMid);
}

void MeshMutator3D::clearCurveSegments()
{
    geometry_.curveSegmentManager_.clear();
}

// ========== Constrained Subfacet Operations ==========

void MeshMutator3D::addConstrainedSubfacet(const ConstrainedSubfacet3D& subfacet)
{
    geometry_.addConstrainedSubfacetInternal(subfacet);
}

void MeshMutator3D::removeConstrainedSubfacet(size_t nodeId1, size_t nodeId2, size_t nodeId3)
{
    geometry_.removeConstrainedSubfacetInternal(nodeId1, nodeId2, nodeId3);
}

void MeshMutator3D::replaceConstrainedSubfacet(const ConstrainedSubfacet3D& oldFacet,
                                                 const std::vector<ConstrainedSubfacet3D>& newFacets)
{
    geometry_.replaceConstrainedSubfacetInternal(oldFacet, newFacets);
}

void MeshMutator3D::clearConstrainedSubfacets()
{
    geometry_.clearConstrainedSubfacetsInternal();
}

void MeshMutator3D::validateNodeRemoval(size_t nodeId) const
{
    if (connectivity_ && !connectivity_->canRemoveNode(nodeId))
    {
        const auto& elements = connectivity_->getNodeElements(nodeId);
        OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                         OpenLoom::MeshException::ErrorCode::INVALID_OPERATION,
                         "Cannot remove node " + std::to_string(nodeId) +
                             ": still referenced by " + std::to_string(elements.size()) + " element(s)");
    }
}

} // namespace Meshing