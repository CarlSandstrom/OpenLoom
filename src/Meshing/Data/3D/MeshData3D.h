#pragma once
#include "../Base/IElement.h"
#include "../2D/MeshData2D.h"
#include "Node3D.h"
#include <memory>
#include <unordered_map>
#include <vector>

namespace Meshing
{

class MeshData3D
{
public:
    MeshData3D();
    // Temporary: Constructor for MeshConnectivity usage
    // TODO: Remove once MeshConnectivity supports MeshData2D directly
    explicit MeshData3D(const MeshData2D& mesh2D);

    // Read-only access to mesh data
    const std::unordered_map<size_t, std::unique_ptr<Node3D>>& getNodes() const;
    const std::unordered_map<size_t, std::unique_ptr<IElement>>& getElements() const;

    const Node3D* getNode(size_t id) const;
    const IElement* getElement(size_t id) const;

    size_t getNodeCount() const;
    size_t getElementCount() const;

    // Internal access for operations classes (friends)
    friend class MeshMutator3D;

private:
    std::unordered_map<size_t, std::unique_ptr<Node3D>> nodes_;
    std::unordered_map<size_t, std::unique_ptr<IElement>> elements_;

    // Private methods for friend classes
    void addNodeInternal(size_t id, std::unique_ptr<Node3D> node);
    void addElementInternal(size_t id, std::unique_ptr<IElement> element);
    void removeNodeInternal(size_t id);
    void removeElementInternal(size_t id);
    Node3D* getNodeMutable(size_t id);
};

} // namespace Meshing