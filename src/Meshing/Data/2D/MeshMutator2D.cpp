#include "MeshMutator2D.h"

#include "Common/Exceptions/MeshException.h"
#include "Node2D.h"

namespace Meshing
{

MeshMutator2D::MeshMutator2D(MeshData2D& meshData) :
    meshData_(meshData)
{
}

size_t MeshMutator2D::addNode(const Point2D& coordinates)
{
    auto node = std::make_unique<Node2D>(coordinates);
    size_t id = meshData_.addNodeInternal(std::move(node));
    return id;
}

size_t MeshMutator2D::addBoundaryNode(const Point2D& coordinates, const std::vector<std::string>& geometryIds)
{
    size_t id = meshData_.addNodeInternal(std::make_unique<Node2D>(coordinates));
    meshData_.setNodeGeometryIdsInternal(id, geometryIds);
    return id;
}

size_t MeshMutator2D::addElement(std::unique_ptr<IElement> element)
{
    size_t id = meshData_.addElementInternal(std::move(element));
    return id;
}

void MeshMutator2D::removeElement(size_t id)
{
    meshData_.removeElementInternal(id);
}

void MeshMutator2D::setCurveSegmentManager(CurveSegmentManager manager)
{
    meshData_.setCurveSegmentManagerInternal(std::move(manager));
}

std::pair<size_t, size_t> MeshMutator2D::splitCurveSegment(size_t nodeId1, size_t nodeId2,
                                                             size_t newNodeId, double tMid)
{
    return meshData_.splitCurveSegmentInternal(nodeId1, nodeId2, newNodeId, tMid);
}

} // namespace Meshing
