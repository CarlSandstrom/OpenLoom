#pragma once
#include "../Connectivity/FaceKey.h"
#include "../Operations/ITransactionListener.h"
#include "Element.h"
#include "Node.h"
#include <memory>
#include <unordered_map>
#include <vector>

namespace Meshing
{

class MeshData
{
public:
    MeshData();

    // Mesh operations
    size_t addNode(const std::array<double, 3>& coordinates);
    void moveNode(size_t id, const std::array<double, 3>& newCoords);
    void removeNode(size_t id);
    size_t getNodeCount() const;
    const Node* getNode(size_t id) const;

    void removeElement(size_t id);
    size_t addElement(std::unique_ptr<Element> element);
    size_t getElementCount() const;
    const Element* getElement(size_t id) const;

    // Connectivity queries
    const std::vector<size_t>& getNodeElements(size_t nodeId) const;
    const std::pair<size_t, size_t>& getFaceElements(const FaceKey& face) const;

    // Transaction support
    void setTransactionListener(ITransactionListener* listener);
    void clearTransactionListener();

    // Public method for transaction to restore elements
    void restoreElement(size_t id, std::unique_ptr<Element> element);
    void restoreNode(size_t id, const std::array<double, 3>& coordinates);

private:
    std::unordered_map<size_t, std::unique_ptr<Node>> nodes_;
    std::unordered_map<size_t, std::unique_ptr<Element>> elements_;
    size_t nextNodeId_ = 0;
    size_t nextElementId_ = 0;

    std::unordered_map<size_t, std::vector<size_t>> nodeToElements_;
    std::unordered_map<FaceKey, std::pair<size_t, size_t>, FaceKeyHash> faceToElements_;

    // Transaction listener (optional)
    ITransactionListener* transactionListener_ = nullptr;

    void addElementToConnectivity_(size_t elementId);
    void removeElementFromConnectivity_(size_t elementId);
};

} // namespace Meshing