#pragma once
#include "../Data/MeshMutator3D.h"
#include "ITransactionListener.h"
#include <memory>
#include <vector>

namespace Meshing
{

class MeshTransaction : public ITransactionListener
{
public:
    explicit MeshTransaction(MeshMutator3D* operations);
    ~MeshTransaction();

    void begin();
    void commit();
    void rollback();
    bool isActive() const { return isActive_; }

    // ITransactionListener implementation
    void onElementAdded(size_t elementId) override;
    void onElementRemoved(size_t elementId, std::unique_ptr<IElement> clone) override;
    void onNodeAdded(size_t nodeId) override;
    void onNodeModified(size_t nodeId, const Point3D& oldCoords) override;
    void onNodeRemoved(size_t nodeId, const Point3D& coords) override;

private:
    struct SavedElement
    {
        size_t id;
        std::unique_ptr<IElement> element;
    };

    struct SavedNode
    {
        size_t id;
        Point3D coordinates;
    };

    MeshMutator3D* meshMutator_;
    bool isActive_;
    bool committed_;

    std::vector<SavedElement> savedElements_;
    std::vector<SavedNode> savedNodes_;
    std::vector<size_t> addedElementIds_;
    std::vector<size_t> addedNodeIds_;
    std::vector<size_t> removedElementIds_;
};

} // namespace Meshing