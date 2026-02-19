#pragma once
#include "../2D/MeshData2D.h"
#include "../Base/IElement.h"
#include "Meshing/Core/3D/General/GeometryStructures3D.h"
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

    // Read-only access to constraints
    const std::vector<ConstrainedSubsegment3D>& getConstrainedSubsegments() const;
    const std::vector<ConstrainedSubfacet3D>& getConstrainedSubfacets() const;
    size_t getConstrainedSubsegmentCount() const;
    size_t getConstrainedSubfacetCount() const;

    // Internal access for operations classes (friends)
    friend class MeshMutator3D;

private:
    std::unordered_map<size_t, std::unique_ptr<Node3D>> nodes_;
    std::unordered_map<size_t, std::unique_ptr<IElement>> elements_;
    std::vector<ConstrainedSubsegment3D> constrainedSubsegments_;
    std::vector<ConstrainedSubfacet3D> constrainedSubfacets_;

    // Private methods for friend classes
    void addNodeInternal(size_t id, std::unique_ptr<Node3D> node);
    void addElementInternal(size_t id, std::unique_ptr<IElement> element);
    void removeNodeInternal(size_t id);
    void removeElementInternal(size_t id);
    Node3D* getNodeMutable(size_t id);

    void addConstrainedSubsegmentInternal(const ConstrainedSubsegment3D& subsegment);
    void removeConstrainedSubsegmentInternal(size_t nodeId1, size_t nodeId2);
    void replaceConstrainedSubsegmentInternal(const ConstrainedSubsegment3D& oldSeg,
                                              const ConstrainedSubsegment3D& newSeg1,
                                              const ConstrainedSubsegment3D& newSeg2);
    void clearConstrainedSubsegmentsInternal();

    void addConstrainedSubfacetInternal(const ConstrainedSubfacet3D& subfacet);
    void removeConstrainedSubfacetInternal(size_t nodeId1, size_t nodeId2, size_t nodeId3);
    void replaceConstrainedSubfacetInternal(const ConstrainedSubfacet3D& oldFacet,
                                            const std::vector<ConstrainedSubfacet3D>& newFacets);
    void clearConstrainedSubfacetsInternal();
};

} // namespace Meshing