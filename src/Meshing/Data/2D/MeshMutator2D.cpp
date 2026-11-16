#include "Meshing/Data/2D/MeshMutator2D.h"

#include "Meshing/Data/2D/Node2D.h"

namespace Meshing
{

MeshMutator2D::MeshMutator2D(MeshData2D& meshData2D) :
    meshData2D_(meshData2D)
{
}

size_t MeshMutator2D::addNode(const Point2D& coordinates)
{
    return meshData2D_.addNodeInternal(std::make_unique<Node2D>(coordinates));
}

size_t MeshMutator2D::addBoundaryNode(const Point2D& coordinates, const std::vector<std::string>& geometryIds)
{
    auto id = addNode(coordinates);
    meshData2D_.setNodeGeometryIdsInternal(id, geometryIds);
    return id;
}

size_t MeshMutator2D::addElement(std::unique_ptr<IElement> element)
{
    return meshData2D_.addElementInternal(std::move(element));
}

void MeshMutator2D::removeElement(size_t id)
{
    meshData2D_.removeElementInternal(id);
}

void MeshMutator2D::setCurveSegmentManager(CurveSegmentManager curveSegmentManager)
{
    meshData2D_.setCurveSegmentManagerInternal(std::move(curveSegmentManager));
}

std::pair<size_t, size_t> MeshMutator2D::splitCurveSegment(size_t nodeId1, size_t nodeId2,
                                                           size_t newNodeId, double tMid)
{
    return meshData2D_.splitCurveSegmentInternal(nodeId1, nodeId2, newNodeId, tMid);
}

} // namespace Meshing
