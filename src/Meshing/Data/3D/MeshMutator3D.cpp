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

size_t MeshMutator3D::addNode(const Point3D& coordinates, double weight)
{
    size_t id = nextNodeId_++;

    auto node = std::make_unique<Node3D>(coordinates);
    node->setWeight(weight);
    geometry_.addNodeInternal(id, std::move(node));

    return id;
}

size_t MeshMutator3D::addBoundaryNode(const Point3D& coordinates,
                                      const std::vector<std::string>& geometryIds,
                                      double weight)
{
    size_t id = nextNodeId_++;

    auto node = std::make_unique<Node3D>(coordinates);
    node->setWeight(weight);
    geometry_.addNodeInternal(id, std::move(node));
    geometry_.setNodeGeometryIdsInternal(id, geometryIds);

    return id;
}

void MeshMutator3D::moveNode(size_t id, const Point3D& newCoords)
{
    Node3D* node = geometry_.getNodeMutable(id);
    if (!node)
    {
        throw OpenLoom::MeshEntityNotFoundException("Node", id, std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

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

    geometry_.removeNodeInternal(id);
}

size_t MeshMutator3D::addElement(std::unique_ptr<IElement> element)
{
    size_t id = nextElementId_++;

    geometry_.addElementInternal(id, std::move(element));

    return id;
}

void MeshMutator3D::removeElement(size_t id)
{
    const IElement* element = geometry_.getElement(id);
    if (!element)
    {
        throw OpenLoom::MeshEntityNotFoundException("Element", id, std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    geometry_.removeElementInternal(id);
}

void MeshMutator3D::setBoundingNodeIds(const std::array<size_t, 4>& boundingNodeIds)
{
    geometry_.setBoundingNodeIdsInternal(boundingNodeIds);
}

void MeshMutator3D::clearBoundingNodeIds()
{
    geometry_.clearBoundingNodeIdsInternal();
}

// ========== Curve Segment Operations ==========

void MeshMutator3D::setCurveSegmentManager(CurveSegmentManager manager)
{
    geometry_.curveSegmentManager_ = std::move(manager);
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