#include "MeshData3D.h"
#include <algorithm>
#include <array>
#include <stdexcept>

namespace Meshing
{

MeshData3D::MeshData3D()
{
}

MeshData3D::MeshData3D(const MeshData2D& mesh2D)
{
    auto& nodes2D = mesh2D.getNodes();
    auto& elements2D = mesh2D.getElements();

    for (const auto& [id, node2D] : nodes2D)
    {
        const Point2D& coords2D = node2D->getCoordinates();
        Point3D coords3D(coords2D.x(), coords2D.y(), 0.0);

        // Copy edge parameters and geometry IDs from 2D node
        const auto& edgeParams = node2D->getEdgeParameters();
        const auto& geometryIds = node2D->getGeometryIds();

        auto node3D = std::make_unique<Node3D>(coords3D, edgeParams, geometryIds);
        addNodeInternal(id, std::move(node3D));
    }

    for (const auto& [id, element2D] : elements2D)
    {
        // Assuming IElement can be cloned for 3D usage
        auto element3D = element2D->clone();
        addElementInternal(id, std::move(element3D));
    }
}

const std::unordered_map<size_t, std::unique_ptr<Node3D>>& MeshData3D::getNodes() const
{
    return nodes_;
}

const std::unordered_map<size_t, std::unique_ptr<IElement>>& MeshData3D::getElements() const
{
    return elements_;
}

const Node3D* MeshData3D::getNode(size_t id) const
{
    auto it = nodes_.find(id);
    return (it != nodes_.end()) ? it->second.get() : nullptr;
}

const IElement* MeshData3D::getElement(size_t id) const
{
    auto it = elements_.find(id);
    return (it != elements_.end()) ? it->second.get() : nullptr;
}

size_t MeshData3D::getNodeCount() const
{
    return nodes_.size();
}

size_t MeshData3D::getElementCount() const
{
    return elements_.size();
}

void MeshData3D::addNodeInternal(size_t id, std::unique_ptr<Node3D> node)
{
    nodes_[id] = std::move(node);
}

void MeshData3D::addElementInternal(size_t id, std::unique_ptr<IElement> element)
{
    elements_[id] = std::move(element);
}

void MeshData3D::removeNodeInternal(size_t id)
{
    nodes_.erase(id);
}

void MeshData3D::removeElementInternal(size_t id)
{
    elements_.erase(id);
}

Node3D* MeshData3D::getNodeMutable(size_t id)
{
    auto it = nodes_.find(id);
    return (it != nodes_.end()) ? it->second.get() : nullptr;
}

const std::vector<ConstrainedSubsegment3D>& MeshData3D::getConstrainedSubsegments() const
{
    return constrainedSubsegments_;
}

const std::vector<ConstrainedSubfacet3D>& MeshData3D::getConstrainedSubfacets() const
{
    return constrainedSubfacets_;
}

size_t MeshData3D::getConstrainedSubsegmentCount() const
{
    return constrainedSubsegments_.size();
}

size_t MeshData3D::getConstrainedSubfacetCount() const
{
    return constrainedSubfacets_.size();
}

void MeshData3D::addConstrainedSubsegmentInternal(const ConstrainedSubsegment3D& subsegment)
{
    constrainedSubsegments_.push_back(subsegment);
}

void MeshData3D::removeConstrainedSubsegmentInternal(size_t nodeId1, size_t nodeId2)
{
    auto it = std::remove_if(constrainedSubsegments_.begin(), constrainedSubsegments_.end(),
        [nodeId1, nodeId2](const ConstrainedSubsegment3D& seg)
        {
            return (seg.nodeId1 == nodeId1 && seg.nodeId2 == nodeId2) ||
                   (seg.nodeId1 == nodeId2 && seg.nodeId2 == nodeId1);
        });
    constrainedSubsegments_.erase(it, constrainedSubsegments_.end());
}

void MeshData3D::replaceConstrainedSubsegmentInternal(const ConstrainedSubsegment3D& oldSeg,
                                                        const ConstrainedSubsegment3D& newSeg1,
                                                        const ConstrainedSubsegment3D& newSeg2)
{
    for (auto it = constrainedSubsegments_.begin(); it != constrainedSubsegments_.end(); ++it)
    {
        if (it->nodeId1 == oldSeg.nodeId1 && it->nodeId2 == oldSeg.nodeId2)
        {
            *it = newSeg1;
            constrainedSubsegments_.push_back(newSeg2);
            return;
        }
    }
}

void MeshData3D::clearConstrainedSubsegmentsInternal()
{
    constrainedSubsegments_.clear();
}

void MeshData3D::addConstrainedSubfacetInternal(const ConstrainedSubfacet3D& subfacet)
{
    constrainedSubfacets_.push_back(subfacet);
}

void MeshData3D::removeConstrainedSubfacetInternal(size_t nodeId1, size_t nodeId2, size_t nodeId3)
{
    auto it = std::remove_if(constrainedSubfacets_.begin(), constrainedSubfacets_.end(),
        [nodeId1, nodeId2, nodeId3](const ConstrainedSubfacet3D& facet)
        {
            std::array<size_t, 3> a = {facet.nodeId1, facet.nodeId2, facet.nodeId3};
            std::array<size_t, 3> b = {nodeId1, nodeId2, nodeId3};
            std::sort(a.begin(), a.end());
            std::sort(b.begin(), b.end());
            return a == b;
        });
    constrainedSubfacets_.erase(it, constrainedSubfacets_.end());
}

void MeshData3D::replaceConstrainedSubfacetInternal(const ConstrainedSubfacet3D& oldFacet,
                                                      const std::vector<ConstrainedSubfacet3D>& newFacets)
{
    for (auto it = constrainedSubfacets_.begin(); it != constrainedSubfacets_.end(); ++it)
    {
        if (it->nodeId1 == oldFacet.nodeId1 &&
            it->nodeId2 == oldFacet.nodeId2 &&
            it->nodeId3 == oldFacet.nodeId3)
        {
            *it = newFacets[0];
            for (size_t i = 1; i < newFacets.size(); ++i)
            {
                constrainedSubfacets_.push_back(newFacets[i]);
            }
            return;
        }
    }
}

void MeshData3D::clearConstrainedSubfacetsInternal()
{
    constrainedSubfacets_.clear();
}

} // namespace Meshing