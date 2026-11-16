#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Common/Exceptions/MeshException.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/Base/MeshConnectivity.h"

#include <algorithm>

namespace Meshing
{

MeshMutator3D::MeshMutator3D(MeshData3D& meshData3D) :
    meshData3D_(meshData3D)
{
    // Continue numbering after the ids already in the mesh so new ids never collide.
    for (const auto& [id, node] : meshData3D_.getNodes())
    {
        nextNodeId_ = std::max(nextNodeId_, id.value() + 1);
    }
    for (const auto& [id, element] : meshData3D_.getElements())
    {
        nextElementId_ = std::max(nextElementId_, id.value() + 1);
    }
}

void MeshMutator3D::setConnectivity(MeshConnectivity* connectivity)
{
    connectivity_ = connectivity;
}

NodeId MeshMutator3D::addNode(const Point3D& coordinates, double weight)
{
    NodeId id{nextNodeId_++};

    auto node = std::make_unique<Node3D>(coordinates);
    node->setWeight(weight);
    meshData3D_.addNodeInternal(id, std::move(node));

    return id;
}

NodeId MeshMutator3D::addBoundaryNode(const Point3D& coordinates,
                                      const std::vector<std::string>& geometryIds,
                                      double weight)
{
    auto id = addNode(coordinates, weight);
    meshData3D_.setNodeGeometryIdsInternal(id, geometryIds);

    return id;
}

void MeshMutator3D::moveNode(NodeId id, const Point3D& newCoordinates)
{
    auto node = meshData3D_.getNodeMutable(id);
    if (!node)
    {
        throw OpenLoom::MeshEntityNotFoundException("Node", id.value(), std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    node->setCoordinates(newCoordinates);
}

void MeshMutator3D::removeNode(NodeId id)
{
    if (!meshData3D_.getNode(id))
    {
        throw OpenLoom::MeshEntityNotFoundException("Node", id.value(), std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }
    if (connectivity_ && !connectivity_->canRemoveNode(id))
    {
        auto elementCount = connectivity_->getNodeElements(id).size();
        OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                            OpenLoom::MeshException::ErrorCode::INVALID_OPERATION,
                            "Cannot remove node " + std::to_string(id.value()) + ": still referenced by " +
                                std::to_string(elementCount) + " element(s)");
    }

    meshData3D_.removeNodeInternal(id);
}

ElementId MeshMutator3D::addElement(std::unique_ptr<IElement> element)
{
    ElementId id{nextElementId_++};

    meshData3D_.addElementInternal(id, std::move(element));

    return id;
}

void MeshMutator3D::removeElement(ElementId id)
{
    if (!meshData3D_.getElement(id))
    {
        throw OpenLoom::MeshEntityNotFoundException("Element", id.value(), std::string(__FILE__) + ":" + std::to_string(__LINE__));
    }

    meshData3D_.removeElementInternal(id);
}

void MeshMutator3D::setBoundingNodeIds(const std::array<NodeId, 4>& boundingNodeIds)
{
    meshData3D_.setBoundingNodeIdsInternal(boundingNodeIds);
}

void MeshMutator3D::clearBoundingNodeIds()
{
    meshData3D_.clearBoundingNodeIdsInternal();
}

void MeshMutator3D::setCurveSegmentManager(CurveSegmentManager curveSegmentManager)
{
    meshData3D_.curveSegmentManager_ = std::move(curveSegmentManager);
}

} // namespace Meshing