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

    // When set, removeNode() refuses a node still referenced by an element.
    void setConnectivity(MeshConnectivity* connectivity);

    // weight is the node's regular-triangulation weight, 0 for an ordinary node.
    size_t addNode(const Point3D& coordinates, double weight = 0.0);
    size_t addBoundaryNode(const Point3D& coordinates,
                           const std::vector<std::string>& geometryIds,
                           double weight = 0.0);
    void moveNode(size_t id, const Point3D& newCoordinates);
    void removeNode(size_t id);

    size_t addElement(std::unique_ptr<IElement> element);
    void removeElement(size_t id);

    // Nodes of the bounding (super-)tetrahedron
    void setBoundingNodeIds(const std::array<size_t, 4>& boundingNodeIds);
    void clearBoundingNodeIds();

    void setCurveSegmentManager(CurveSegmentManager curveSegmentManager);

private:
    MeshData3D& meshData3D_;
    MeshConnectivity* connectivity_ = nullptr;
    size_t nextNodeId_ = 0;
    size_t nextElementId_ = 0;
};

} // namespace Meshing