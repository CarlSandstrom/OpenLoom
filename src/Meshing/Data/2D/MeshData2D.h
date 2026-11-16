#pragma once

#include "Common/Types.h"
#include "Meshing/Data/2D/Node2D.h"
#include "Meshing/Data/Base/IElement.h"
#include "Meshing/Data/CurveSegmentManager.h"

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace Meshing
{

class MeshData2D
{
public:
    MeshData2D() = default;

    const std::unordered_map<NodeId, std::unique_ptr<Node2D>>& getNodes() const { return nodes_; }
    const std::unordered_map<ElementId, std::unique_ptr<IElement>>& getElements() const { return elements_; }

    const Node2D* getNode(NodeId id) const;
    const IElement* getElement(ElementId id) const;

    size_t getElementCount() const { return elements_.size(); }

    const CurveSegmentManager& getCurveSegmentManager() const { return curveSegmentManager_; }

    friend class MeshMutator2D;

private:
    std::unordered_map<NodeId, std::unique_ptr<Node2D>> nodes_;
    std::unordered_map<ElementId, std::unique_ptr<IElement>> elements_;
    std::unordered_map<NodeId, std::vector<std::string>> nodeGeometryIds_;
    CurveSegmentManager curveSegmentManager_;
    size_t nextNodeId_ = 0;
    size_t nextElementId_ = 0;

    NodeId addNodeInternal(std::unique_ptr<Node2D> node);
    ElementId addElementInternal(std::unique_ptr<IElement> element);
    void removeElementInternal(ElementId id);

    void setNodeGeometryIdsInternal(NodeId nodeId, std::vector<std::string> geometryIds);

    void setCurveSegmentManagerInternal(CurveSegmentManager manager);
    std::pair<SegmentId, SegmentId> splitCurveSegmentInternal(NodeId nodeId1, NodeId nodeId2,
                                                          NodeId newNodeId, double tMid);
};

} // namespace Meshing
