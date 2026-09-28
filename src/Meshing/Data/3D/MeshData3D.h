#pragma once
#include "../Base/IElement.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "Node3D.h"
#include <array>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace Meshing
{

class MeshData3D
{
public:
    MeshData3D();

    // Read-only access to mesh data
    const std::unordered_map<size_t, std::unique_ptr<Node3D>>& getNodes() const;
    const std::unordered_map<size_t, std::unique_ptr<IElement>>& getElements() const;

    const Node3D* getNode(size_t id) const;
    const IElement* getElement(size_t id) const;

    size_t getNodeCount() const;
    size_t getElementCount() const;

    // Read-only access to constraints
    const CurveSegmentManager& getCurveSegmentManager() const;

    // Geometry ID association (boundary node metadata)
    const std::vector<std::string>& getGeometryIds(size_t nodeId) const;

    // Node IDs of the bounding (super-)tetrahedron currently resident in the
    // mesh, if any. Set while the bounding tetrahedron is present (see
    // MeshOperations3D::createBoundingTetrahedron and AmbientTetrahedronRemover),
    // used by VtkExporter to tag those cells for filtering in ParaView.
    const std::optional<std::array<size_t, 4>>& getBoundingNodeIds() const;

    // Internal access for operations classes (friends)
    friend class MeshMutator3D;

private:
    std::unordered_map<size_t, std::unique_ptr<Node3D>> nodes_;
    std::unordered_map<size_t, std::unique_ptr<IElement>> elements_;
    std::unordered_map<size_t, std::vector<std::string>> nodeGeometryIds_;
    std::optional<std::array<size_t, 4>> boundingNodeIds_;
    CurveSegmentManager curveSegmentManager_;

    // Private methods for friend classes
    void addNodeInternal(size_t id, std::unique_ptr<Node3D> node);
    void addElementInternal(size_t id, std::unique_ptr<IElement> element);
    void removeNodeInternal(size_t id);
    void removeElementInternal(size_t id);
    Node3D* getNodeMutable(size_t id);

    void setNodeGeometryIdsInternal(size_t nodeId, std::vector<std::string> ids);

    void setBoundingNodeIdsInternal(const std::array<size_t, 4>& boundingNodeIds);
    void clearBoundingNodeIdsInternal();
};

} // namespace Meshing