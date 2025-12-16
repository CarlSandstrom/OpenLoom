#pragma once
#include "../Connectivity/FaceKey.h"
#include "MeshData3D.h"
#include <unordered_map>
#include <vector>

namespace Meshing
{

class MeshConnectivity
{
public:
    explicit MeshConnectivity(const MeshData& geometry);

    // Connectivity queries
    const std::vector<size_t>& getNodeElements(size_t nodeId) const;
    const std::pair<size_t, size_t>& getFaceElements(const FaceKey& face) const;

    // Rebuild connectivity when geometry changes
    void rebuildConnectivity();

    // Check if a node can be safely removed
    bool canRemoveNode(size_t nodeId) const;

private:
    const MeshData& geometry_;

    std::unordered_map<size_t, std::vector<size_t>> nodeToElements_;
    std::unordered_map<FaceKey, std::pair<size_t, size_t>, FaceKeyHash> faceToElements_;

    void buildNodeToElementsMap();
    void buildFaceToElementsMap();
    void addElementToConnectivity(size_t elementId);
    void removeElementFromConnectivity(size_t elementId);
};

} // namespace Meshing