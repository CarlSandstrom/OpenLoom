#pragma once
#include "../Base/IElement.h"
#include "../Operations/ITransactionListener.h"
#include "MeshData3D.h"
#include "Meshing/Core/3D/GeometryStructures3D.h"
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

    // Node operations
    size_t addNode(const Point3D& coordinates);
    size_t addBoundaryNode(const Point3D& coordinates,
                           const std::vector<double>& edgeParameters,
                           const std::vector<std::string>& geometryIds);
    void moveNode(size_t id, const Point3D& newCoords);
    void removeNode(size_t id);

    // Element operations
    size_t addElement(std::unique_ptr<IElement> element);
    void removeElement(size_t id);

    // Constrained subsegment operations
    void addConstrainedSubsegment(const ConstrainedSubsegment3D& subsegment);
    void removeConstrainedSubsegment(size_t nodeId1, size_t nodeId2);
    void replaceConstrainedSubsegment(const ConstrainedSubsegment3D& oldSeg,
                                      const ConstrainedSubsegment3D& newSeg1,
                                      const ConstrainedSubsegment3D& newSeg2);
    void clearConstrainedSubsegments();

    // Constrained subfacet operations
    void addConstrainedSubfacet(const ConstrainedSubfacet3D& subfacet);
    void removeConstrainedSubfacet(size_t nodeId1, size_t nodeId2, size_t nodeId3);
    void replaceConstrainedSubfacet(const ConstrainedSubfacet3D& oldFacet,
                                    const std::vector<ConstrainedSubfacet3D>& newFacets);
    void clearConstrainedSubfacets();

    // Transaction support
    void setTransactionListener(ITransactionListener* listener);
    void clearTransactionListener();

    // Public methods for transaction to restore elements/nodes
    void restoreElement(size_t id, std::unique_ptr<IElement> element);
    void restoreNode(size_t id, const Point3D& coordinates);

private:
    MeshData3D& geometry_;
    MeshConnectivity* connectivity_ = nullptr; // Optional for validation
    size_t nextNodeId_ = 0;
    size_t nextElementId_ = 0;

    // Transaction listener (optional)
    ITransactionListener* transactionListener_ = nullptr;

    void validateNodeRemoval(size_t nodeId) const;
};

} // namespace Meshing