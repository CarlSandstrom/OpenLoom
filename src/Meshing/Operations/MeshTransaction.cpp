#include "MeshTransaction.h"

namespace Meshing
{

MeshTransaction::MeshTransaction(MeshOperations* operations) :
    operations_(operations), isActive_(false), committed_(false)
{
}

MeshTransaction::~MeshTransaction()
{
    if (isActive_ && !committed_)
    {
        rollback();
    }
}

void MeshTransaction::begin()
{
    isActive_ = true;
    committed_ = false;

    savedElements_.clear();
    savedNodes_.clear();
    addedElementIds_.clear();
    addedNodeIds_.clear();
    removedElementIds_.clear();

    // Register as listener
    operations_->setTransactionListener(this);
}

void MeshTransaction::commit()
{
    if (!isActive_) return;

    isActive_ = false;
    committed_ = true;

    // Unregister as listener
    operations_->clearTransactionListener();

    // Clear saved data
    savedElements_.clear();
    savedNodes_.clear();
    addedElementIds_.clear();
    addedNodeIds_.clear();
    removedElementIds_.clear();
}

void MeshTransaction::rollback()
{
    if (!isActive_) return;

    // Unregister first to avoid recording rollback operations
    operations_->clearTransactionListener();

    // 1. Remove newly added elements
    for (size_t id : addedElementIds_)
    {
        operations_->removeElement(id);
    }

    // 2. Restore removed elements
    for (auto& saved : savedElements_)
    {
        operations_->restoreElement(saved.id, std::move(saved.element));
    }

    // 3. Remove newly added nodes
    for (size_t id : addedNodeIds_)
    {
        operations_->removeNode(id);
    }

    // 4. Restore modified nodes
    for (const auto& saved : savedNodes_)
    {
        operations_->restoreNode(saved.id, saved.coordinates);
    }

    isActive_ = false;
    committed_ = false;
}

// ITransactionListener callbacks
void MeshTransaction::onElementAdded(size_t elementId)
{
    addedElementIds_.push_back(elementId);
}

void MeshTransaction::onElementRemoved(size_t elementId, std::unique_ptr<IElement> clone)
{
    savedElements_.push_back({elementId, std::move(clone)});
    removedElementIds_.push_back(elementId);
}

void MeshTransaction::onNodeAdded(size_t nodeId)
{
    addedNodeIds_.push_back(nodeId);
}

void MeshTransaction::onNodeModified(size_t nodeId, const std::array<double, 3>& oldCoords)
{
    savedNodes_.push_back({nodeId, oldCoords});
}

void MeshTransaction::onNodeRemoved(size_t nodeId, const std::array<double, 3>& coords)
{
    // Could save node data if needed for restoration
}

// ScopedTransaction implementation
ScopedTransaction::ScopedTransaction(MeshOperations* operations) :
    transaction_(operations)
{
    transaction_.begin();
}

ScopedTransaction::~ScopedTransaction()
{
    if (transaction_.isActive())
    {
        transaction_.rollback();
    }
}

void ScopedTransaction::commit()
{
    transaction_.commit();
}

} // namespace Meshing