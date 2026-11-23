#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/General/MeshQueries2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include <memory>
#include <optional>
#include <unordered_set>
#include <vector>

namespace Geometry2D
{
class IEdge2D;
}

namespace Meshing
{

class MeshMutator2D;
class PeriodicMeshData2D;

class MeshOperations2D
{
public:
    explicit MeshOperations2D(MeshData2D& meshData);
    MeshOperations2D(MeshData2D& meshData, PeriodicMeshData2D* periodicData);

    /// Returns nullopt, with the mesh untouched, when constrained edges leave the point
    /// no conflicting triangle.
    std::optional<NodeId> insertVertexBowyerWatson(const Point2D& point);

    bool removeTrianglesContainingNode(NodeId nodeId);

    bool enforceEdge(NodeId nodeId1, NodeId nodeId2);

    std::optional<NodeId> splitConstrainedSegment(
        const CurveSegment& segment,
        const Geometry2D::IEdge2D& parentEdge);

    std::vector<ElementId> removeExteriorTriangles(const std::unordered_set<ElementId>& interiorTriangles);

    std::vector<ElementId> classifyAndRemoveExteriorTriangles();

    MeshMutator2D& getMutator() { return *mutator_; }
    const MeshMutator2D& getMutator() const { return *mutator_; }

    MeshQueries2D& getQueries() { return queries_; }
    const MeshQueries2D& getQueries() const { return queries_; }

private:
    std::vector<ElementId> splitTrianglesAtEdge(NodeId edgeNode1, NodeId edgeNode2, NodeId midNodeId);
    std::optional<NodeId> extendMeshAcrossChord(const CurveSegment& segment, const Point2D& midPoint, double midParameter);
    void removeTriangleOnChord(NodeId chordNode1, NodeId chordNode2, NodeId apexNodeId);
    void lawsonFlip(const std::vector<ElementId>& newTriangleIds);

    MeshData2D& meshData_;
    PeriodicMeshData2D* periodicData_ = nullptr;
    MeshQueries2D queries_;
    std::unique_ptr<MeshMutator2D> mutator_;
};

} // namespace Meshing
