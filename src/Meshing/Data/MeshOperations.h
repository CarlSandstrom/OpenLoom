#pragma once
#include "../Operations/ITransactionListener.h"
#include "Element.h"
#include "MeshGeometry.h"
#include <array>
#include <memory>

namespace Meshing
{

class MeshConnectivity; // Forward declaration

class MeshOperations
{
public:
    explicit MeshOperations(MeshGeometry& geometry);

    // Optional: Set connectivity for validation during operations
    void setConnectivity(MeshConnectivity* connectivity);

    // Node operations
    size_t addNode(const std::array<double, 3>& coordinates);
    void moveNode(size_t id, const std::array<double, 3>& newCoords);
    void removeNode(size_t id);

    // Element operations
    size_t addElement(std::unique_ptr<Element> element);
    void removeElement(size_t id);

    // Transaction support
    void setTransactionListener(ITransactionListener* listener);
    void clearTransactionListener();

    // Public methods for transaction to restore elements/nodes
    void restoreElement(size_t id, std::unique_ptr<Element> element);
    void restoreNode(size_t id, const std::array<double, 3>& coordinates);

private:
    MeshGeometry& geometry_;
    MeshConnectivity* connectivity_ = nullptr; // Optional for validation
    size_t nextNodeId_ = 0;
    size_t nextElementId_ = 0;

    // Transaction listener (optional)
    ITransactionListener* transactionListener_ = nullptr;

    void validateNodeRemoval_(size_t nodeId) const;
};

} // namespace Meshing