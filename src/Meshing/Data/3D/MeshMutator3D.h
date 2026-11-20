#pragma once
#include "Common/Types.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/Base/IElement.h"
#include "Meshing/Data/CurveSegmentManager.h"

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace Meshing
{

class MeshConnectivity;

class MeshMutator3D
{
public:
    explicit MeshMutator3D(MeshData3D& meshData3D);

    // When set, it is kept current as elements are added and removed, and
    // removeNode() refuses a node still referenced by an element.
    void setConnectivity(MeshConnectivity* connectivity);

    // weight is the node's regular-triangulation weight, 0 for an ordinary node.
    NodeId addNode(const Point3D& coordinates, double weight = 0.0);
    NodeId addBoundaryNode(const Point3D& coordinates,
                           const std::vector<std::string>& geometryIds,
                           double weight = 0.0);
    void moveNode(NodeId id, const Point3D& newCoordinates);
    void setNodeWeight(NodeId id, double weight);
    void removeNode(NodeId id);

    ElementId addElement(std::unique_ptr<IElement> element);
    void removeElement(ElementId id);

    // Nodes of the bounding (super-)tetrahedron
    void setBoundingNodeIds(const std::array<NodeId, 4>& boundingNodeIds);
    void clearBoundingNodeIds();

    void setCurveSegmentManager(CurveSegmentManager curveSegmentManager);

private:
    MeshData3D& meshData3D_;
    MeshConnectivity* connectivity_ = nullptr;
    size_t nextNodeId_ = 0;
    size_t nextElementId_ = 0;
};

} // namespace Meshing