#pragma once
#include "IElement.h"
#include "Node3D.h"
#include <memory>
#include <unordered_map>
#include <vector>

namespace Meshing
{

class MeshData
{
public:
    MeshData();

    // Read-only access to mesh data
    const std::unordered_map<size_t, std::unique_ptr<Node3D>>& getNodes() const;
    const std::unordered_map<size_t, std::unique_ptr<IElement>>& getElements() const;

    const Node3D* getNode(size_t id) const;
    const IElement* getElement(size_t id) const;

    size_t getNodeCount() const;
    size_t getElementCount() const;

    // Internal access for operations classes (friends)
    friend class MeshOperations;

private:
    std::unordered_map<size_t, std::unique_ptr<Node3D>> nodes_;
    std::unordered_map<size_t, std::unique_ptr<IElement>> elements_;

    // Private methods for friend classes
    void addNodeInternal_(size_t id, std::unique_ptr<Node3D> node);
    void addElementInternal_(size_t id, std::unique_ptr<IElement> element);
    void removeNodeInternal_(size_t id);
    void removeElementInternal_(size_t id);
    Node3D* getNodeMutable_(size_t id);
};

} // namespace Meshing