#pragma once
#include "../Base/IElement.h"
#include "MeshData3D.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "Common/Types.h"

namespace Meshing
{

class MeshConnectivity; // Forward declaration

class MeshMutator3D
{
public:
    explicit MeshMutator3D(MeshData3D& geometry);

    // Optional: Set connectivity for validation during operations
    void setConnectivity(MeshConnectivity* connectivity);

    // Node operations. weight is the node's regular-triangulation weight
    // (0 for an ordinary, unweighted node -- see Node3D::getWeight() and
    // RegularPredicates3D, OPE-176).
    size_t addNode(const Point3D& coordinates, double weight = 0.0);
    size_t addBoundaryNode(const Point3D& coordinates,
                           const std::vector<std::string>& geometryIds,
                           double weight = 0.0);
    void moveNode(size_t id, const Point3D& newCoordinates);
    void removeNode(size_t id);

    // Element operations
    size_t addElement(std::unique_ptr<IElement> element);
    void removeElement(size_t id);

    // Bounding (super-)tetrahedron tracking
    void setBoundingNodeIds(const std::array<size_t, 4>& boundingNodeIds);
    void clearBoundingNodeIds();

    // Curve segment operations
    void setCurveSegmentManager(CurveSegmentManager manager);

private:
    MeshData3D& geometry_;
    MeshConnectivity* connectivity_ = nullptr; // Optional for validation
    size_t nextNodeId_ = 0;
    size_t nextElementId_ = 0;

    void validateNodeRemoval(size_t nodeId) const;
};

} // namespace Meshing