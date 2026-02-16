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

size_t MeshMutator2D::addBoundaryNode(const Point2D& coordinates, const std::vector<double>& edgeParameters, const std::vector<std::string>& geometryIds)
{
    auto node = std::make_unique<Node2D>(coordinates, edgeParameters, geometryIds);
    size_t id = meshData_.addNodeInternal(std::move(node));
    return id;
}

void MeshMutator2D::moveNode(size_t id, const Point2D& newCoords)
{
    Node2D* node = meshData_.getNodeMutable(id);
    if (node == nullptr)
    {
        CMESH_THROW_CODE(cMesh::MeshException,
                         cMesh::MeshException::ErrorCode::NODE_NOT_FOUND,
                         "Cannot move node " + std::to_string(id) + ": node not found");
    }
    node->setCoordinates(newCoords);
}

void MeshMutator2D::removeNode(size_t id)
{
    meshData_.removeNodeInternal(id);
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

void MeshMutator2D::addConstrainedSegment(const ConstrainedSegment2D& segment)
{
    meshData_.addConstrainedSegmentInternal(segment);
}

void MeshMutator2D::removeConstrainedSegment(size_t nodeId1, size_t nodeId2)
{
    meshData_.removeConstrainedSegmentInternal(nodeId1, nodeId2);
}

void MeshMutator2D::replaceConstrainedSegment(const ConstrainedSegment2D& oldSegment,
                                               const ConstrainedSegment2D& newSeg1,
                                               const ConstrainedSegment2D& newSeg2)
{
    meshData_.replaceConstrainedSegmentInternal(oldSegment, newSeg1, newSeg2);
}

void MeshMutator2D::clearConstrainedSegments()
{
    meshData_.clearConstrainedSegmentsInternal();
}

} // namespace Meshing
