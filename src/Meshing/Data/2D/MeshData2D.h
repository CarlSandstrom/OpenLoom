#pragma once

#include "../Base/IElement.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Node2D.h"
#include <memory>
#include <unordered_map>
#include <vector>

namespace Meshing
{

/**
 * @brief Storage for 2D mesh data (nodes and elements in parametric space)
 */
class MeshData2D
{
public:
    MeshData2D() = default;

    // Read-only access to mesh data
    const std::unordered_map<size_t, std::unique_ptr<Node2D>>& getNodes() const { return nodes_; }
    const std::unordered_map<size_t, std::unique_ptr<IElement>>& getElements() const { return elements_; }

    const Node2D* getNode(size_t id) const;
    const IElement* getElement(size_t id) const;

    size_t getNodeCount() const { return nodes_.size(); }
    size_t getElementCount() const { return elements_.size(); }

    const std::vector<ConstrainedSegment2D>& getConstrainedSegments() const { return constrainedSegments_; }
    size_t getConstrainedSegmentCount() const { return constrainedSegments_.size(); }

    // Internal access for operations classes (friends)
    friend class MeshMutator2D;

private:
    std::unordered_map<size_t, std::unique_ptr<Node2D>> nodes_;
    std::unordered_map<size_t, std::unique_ptr<IElement>> elements_;
    std::vector<ConstrainedSegment2D> constrainedSegments_;
    size_t nextNodeId_ = 0;
    size_t nextElementId_ = 0;

    // Private methods for friend classes
    size_t addNodeInternal(std::unique_ptr<Node2D> node);
    size_t addElementInternal(std::unique_ptr<IElement> element);
    void removeNodeInternal(size_t id);
    void removeElementInternal(size_t id);
    Node2D* getNodeMutable(size_t id);

    void addConstrainedSegmentInternal(const ConstrainedSegment2D& segment);
    void removeConstrainedSegmentInternal(size_t nodeId1, size_t nodeId2);
    void replaceConstrainedSegmentInternal(const ConstrainedSegment2D& oldSegment,
                                           const ConstrainedSegment2D& newSeg1,
                                           const ConstrainedSegment2D& newSeg2);
    void clearConstrainedSegmentsInternal();
};

} // namespace Meshing
