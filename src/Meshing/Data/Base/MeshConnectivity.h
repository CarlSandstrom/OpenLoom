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

    // Check if a node can be safely removed
    bool canRemoveNode(NodeId nodeId) const;

private:
    const MeshData3D& geometry_;

    std::unordered_map<NodeId, std::vector<ElementId>> nodeToElements_;
    std::unordered_map<FaceKey, std::pair<ElementId, ElementId>, FaceKeyHash> faceToElements_;

    void buildNodeToElementsMap();
    void buildFaceToElementsMap();
    void addElementToConnectivity(ElementId elementId);
};

} // namespace Meshing