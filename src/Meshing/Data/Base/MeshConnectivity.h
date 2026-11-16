#pragma once
#include "Common/Types.h"
#include "../3D/MeshData3D.h"
#include "../Connectivity/FaceKey.h"
#include <unordered_map>
#include <vector>

namespace Meshing
{

class MeshConnectivity
{
public:
    explicit MeshConnectivity(const MeshData3D& geometry);

    // Connectivity queries
    const std::vector<ElementId>& getNodeElements(NodeId nodeId) const;
    const std::pair<ElementId, ElementId>& getFaceElements(const FaceKey& face) const;

    // Rebuild connectivity when geometry changes
    void rebuildConnectivity();

    /// Records an element already in the mesh data.
    void addElement(ElementId elementId);

    /// Forgets an element. Call before it leaves the mesh data: its nodes are read from there.
    void removeElement(ElementId elementId);

    // Check if a node can be safely removed
    bool canRemoveNode(NodeId nodeId) const;

private:
    const MeshData3D& geometry_;

    std::unordered_map<NodeId, std::vector<ElementId>> nodeToElements_;
    std::unordered_map<FaceKey, std::pair<ElementId, ElementId>, FaceKeyHash> faceToElements_;

};

} // namespace Meshing