#pragma once

#include "Common/Types.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/Base/TriangleElement.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include <array>
#include <map>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Topology2D
{
class Topology2D;
}

namespace Meshing
{

class PeriodicMeshData2D;

class MeshQueries2D
{
public:
    using EdgeKey = std::pair<NodeId, NodeId>;

    struct EdgeKeyHash
    {
        std::size_t operator()(const EdgeKey& key) const
        {
            return std::hash<size_t>{}(key.first) ^ (std::hash<size_t>{}(key.second) << 1);
        }
    };

    using EdgeToTrianglesMap = std::unordered_map<EdgeKey, std::vector<ElementId>, EdgeKeyHash>;

    explicit MeshQueries2D(const MeshData2D& meshData);
    MeshQueries2D(const MeshData2D& meshData, PeriodicMeshData2D* periodicData);

    std::vector<ElementId> findConflictingTriangles(const Point2D& point) const;

    std::vector<std::array<NodeId, 2>> findCavityBoundary(const std::vector<ElementId>& conflictingIndices) const;

    std::vector<ElementId> findIntersectingTriangles(NodeId nodeId1, NodeId nodeId2) const;

    /// Extracts constrained edges from topology as a CurveSegmentManager.
    /// tParameters and geometryIds must be parallel to the discretization points array
    /// (same indexing as cornerIdToPointIndexMap and edgeIdToPointIndicesMap).
    CurveSegmentManager extractConstrainedEdges(
        const Topology2D::Topology2D& topology,
        const std::map<std::string, size_t>& cornerIdToPointIndexMap,
        const std::map<size_t, NodeId>& pointIndexToNodeIdMap,
        const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap,
        const std::vector<std::vector<double>>& tParameters,
        const std::vector<std::vector<std::string>>& geometryIds) const;

    std::vector<CurveSegment> findEncroachedSegments() const;

    std::vector<CurveSegment> findSegmentsEncroachedByPoint(const Point2D& point) const;

    static EdgeKey makeEdgeKey(NodeId nodeId0, NodeId nodeId1)
    {
        return (nodeId0 < nodeId1) ? std::make_pair(nodeId0, nodeId1) : std::make_pair(nodeId1, nodeId0);
    }

    std::vector<ElementId> findTrianglesAdjacentToEdge(NodeId nodeId1, NodeId nodeId2) const;

    bool isBoundaryConstraintEdge(const std::array<NodeId, 2>& edge) const;

    EdgeToTrianglesMap buildEdgeToTrianglesMap() const;

    bool isPointInsideDomain(const Point2D& point) const;

    std::unordered_set<ElementId> classifyTrianglesInteriorExterior() const;

    bool isPointVisibleFromSegment(const Point2D& point, const CurveSegment& segment) const;

private:
    const MeshData2D& meshData_;
    PeriodicMeshData2D* periodicData_ = nullptr;
};

} // namespace Meshing
