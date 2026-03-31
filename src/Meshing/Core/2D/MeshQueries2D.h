#pragma once

#include "Common/Types.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include <array>
#include <map>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Geometry2D
{
class IEdge2D;
}

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
    using EdgeKey = std::pair<size_t, size_t>;

    struct EdgeKeyHash
    {
        std::size_t operator()(const EdgeKey& key) const
        {
            return std::hash<size_t>{}(key.first) ^ (std::hash<size_t>{}(key.second) << 1);
        }
    };

    using EdgeToTrianglesMap = std::unordered_map<EdgeKey, std::vector<size_t>, EdgeKeyHash>;

    explicit MeshQueries2D(const MeshData2D& meshData);
    MeshQueries2D(const MeshData2D& meshData, PeriodicMeshData2D* periodicData);

    std::vector<size_t> findConflictingTriangles(const Point2D& point) const;

    std::vector<std::array<size_t, 2>> findCavityBoundary(const std::vector<size_t>& conflictingIndices) const;

    std::vector<size_t> findIntersectingTriangles(size_t nodeId1, size_t nodeId2) const;

    /// Extracts constrained edges from topology as a CurveSegmentManager.
    /// tParameters and geometryIds must be parallel to the discretization points array
    /// (same indexing as cornerIdToPointIndexMap and edgeIdToPointIndicesMap).
    CurveSegmentManager extractConstrainedEdges(
        const Topology2D::Topology2D& topology,
        const std::map<std::string, size_t>& cornerIdToPointIndexMap,
        const std::map<size_t, size_t>& pointIndexToNodeIdMap,
        const std::map<std::string, std::vector<size_t>>& edgeIdToPointIndicesMap,
        const std::vector<std::vector<double>>& tParameters,
        const std::vector<std::vector<std::string>>& geometryIds) const;

    std::vector<CurveSegment> findEncroachedSegments() const;

    std::vector<CurveSegment> findSegmentsEncroachedByPoint(const Point2D& point) const;

    static std::pair<size_t, size_t> makeEdgeKey(size_t a, size_t b)
    {
        return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
    }

    std::vector<size_t> findTrianglesAdjacentToEdge(size_t nodeId1, size_t nodeId2) const;

    std::optional<std::string> findCommonGeometryId(size_t nodeId1, size_t nodeId2) const;

    bool isBoundaryConstraintEdge(const std::array<size_t, 2>& edgeId) const;

    EdgeToTrianglesMap buildEdgeToTrianglesMap() const;

    bool isPointInsideDomain(const Point2D& point) const;

    std::unordered_set<size_t> classifyTrianglesInteriorExterior() const;

    bool isPointVisibleFromSegment(const Point2D& point, const CurveSegment& segment) const;

private:
    const MeshData2D& meshData_;
    PeriodicMeshData2D* periodicData_ = nullptr;
};

} // namespace Meshing
