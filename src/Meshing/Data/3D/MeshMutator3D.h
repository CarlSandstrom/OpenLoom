#pragma once
#include "../Operations/ITransactionListener.h"
#include "../Base/IElement.h"
#include "MeshData3D.h"
#include <memory>

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
    void moveNode(size_t id, const Point3D& newCoords);
    void removeNode(size_t id);

    // Element operations
    size_t addElement(std::unique_ptr<IElement> element);
    void removeElement(size_t id);

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