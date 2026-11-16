#pragma once

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

    const std::unordered_map<size_t, std::unique_ptr<Node2D>>& getNodes() const { return nodes_; }
    const std::unordered_map<size_t, std::unique_ptr<IElement>>& getElements() const { return elements_; }

    const Node2D* getNode(size_t id) const;
    const IElement* getElement(size_t id) const;

    size_t getElementCount() const { return elements_.size(); }

    const CurveSegmentManager& getCurveSegmentManager() const { return curveSegmentManager_; }

    friend class MeshMutator2D;

private:
    std::unordered_map<size_t, std::unique_ptr<Node2D>> nodes_;
    std::unordered_map<size_t, std::unique_ptr<IElement>> elements_;
    std::unordered_map<size_t, std::vector<std::string>> nodeGeometryIds_;
    CurveSegmentManager curveSegmentManager_;
    size_t nextNodeId_ = 0;
    size_t nextElementId_ = 0;

    size_t addNodeInternal(std::unique_ptr<Node2D> node);
    size_t addElementInternal(std::unique_ptr<IElement> element);
    void removeElementInternal(size_t id);

    void setNodeGeometryIdsInternal(size_t nodeId, std::vector<std::string> geometryIds);

    void setCurveSegmentManagerInternal(CurveSegmentManager manager);
    std::pair<size_t, size_t> splitCurveSegmentInternal(size_t nodeId1, size_t nodeId2,
                                                        size_t newNodeId, double tMid);
};

} // namespace Meshing
