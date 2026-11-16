#include "Meshing/Data/2D/MeshMutator2D.h"

#include "Meshing/Data/2D/Node2D.h"

namespace Meshing
{

MeshMutator2D::MeshMutator2D(MeshData2D& meshData2D) :
    meshData2D_(meshData2D)
{
}

NodeId MeshMutator2D::addNode(const Point2D& coordinates)
{
    return meshData2D_.addNodeInternal(std::make_unique<Node2D>(coordinates));
}

NodeId MeshMutator2D::addBoundaryNode(const Point2D& coordinates, const std::vector<std::string>& geometryIds)
{
    auto id = addNode(coordinates);
    meshData2D_.setNodeGeometryIdsInternal(id, geometryIds);
    return id;
}

ElementId MeshMutator2D::addElement(std::unique_ptr<IElement> element)
{
    return meshData2D_.addElementInternal(std::move(element));
}

void MeshMutator2D::removeElement(ElementId id)
{
    meshData2D_.removeElementInternal(id);
}

void MeshMutator2D::setCurveSegmentManager(CurveSegmentManager curveSegmentManager)
{
    meshData2D_.setCurveSegmentManagerInternal(std::move(curveSegmentManager));
}

std::pair<SegmentId, SegmentId> MeshMutator2D::splitCurveSegment(NodeId nodeId1, NodeId nodeId2,
                                                                 NodeId newNodeId, double tMid)
{
    return meshData2D_.splitCurveSegmentInternal(nodeId1, nodeId2, newNodeId, tMid);
}

} // namespace Meshing
