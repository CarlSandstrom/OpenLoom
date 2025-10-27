#pragma once
#include "../Data/MeshData.h"
#include "ITransactionListener.h"
#include <memory>
#include <vector>

namespace Meshing
{

class MeshTransaction : public ITransactionListener
{
public:
    explicit MeshTransaction(MeshData* mesh);
    ~MeshTransaction();

    void begin();
    void commit();
    void rollback();
    bool isActive() const { return isActive_; }

    // ITransactionListener implementation
    void onElementAdded(size_t elementId) override;
    void onElementRemoved(size_t elementId, std::unique_ptr<Element> clone) override;
    void onNodeAdded(size_t nodeId) override;
    void onNodeModified(size_t nodeId, const std::array<double, 3>& oldCoords) override;
    void onNodeRemoved(size_t nodeId, const std::array<double, 3>& coords) override;

private:
    struct SavedElement
    {
        size_t id;
        std::unique_ptr<Element> element;
    };

    struct SavedNode
    {
        size_t id;
        std::array<double, 3> coordinates;
    };

    MeshData* mesh_;
    bool isActive_;
    bool committed_;

    std::vector<SavedElement> savedElements_;
    std::vector<SavedNode> savedNodes_;
    std::vector<size_t> addedElementIds_;
    std::vector<size_t> addedNodeIds_;
    std::vector<size_t> removedElementIds_;
};

class ScopedTransaction
{
public:
    explicit ScopedTransaction(MeshData* mesh);
    ~ScopedTransaction();
    void commit();

private:
    MeshTransaction transaction_;
};

} // namespace Meshing