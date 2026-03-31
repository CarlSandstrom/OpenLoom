#pragma once

#include "Common/Types.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "Meshing/Core/2D/ElementGeometry2D.h"
#include "Meshing/Core/2D/MeshQueries2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include <array>
#include <map>
#include <memory>
#include <optional>
#include <string>
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

class MeshMutator2D;
class PeriodicMeshData2D;

class MeshOperations2D
{
public:
    explicit MeshOperations2D(MeshData2D& meshData);
    MeshOperations2D(MeshData2D& meshData, PeriodicMeshData2D* periodicData);

    size_t insertVertexBowyerWatson(const Point2D& point,
                                    const std::vector<std::string>& edgeIds = {});

    bool removeTrianglesContainingNode(size_t nodeId);

    bool enforceEdge(size_t nodeId1, size_t nodeId2);

    std::optional<size_t> splitConstrainedSegment(
        const CurveSegment& segment,
        const Geometry2D::IEdge2D& parentEdge);

    std::vector<size_t> removeExteriorTriangles(const std::unordered_set<size_t>& interiorTriangles);

    std::vector<size_t> classifyAndRemoveExteriorTriangles();

    MeshMutator2D& getMutator() { return *mutator_; }
    const MeshMutator2D& getMutator() const { return *mutator_; }

    MeshQueries2D& getQueries() { return queries_; }
    const MeshQueries2D& getQueries() const { return queries_; }

private:
    std::vector<size_t> splitTrianglesAtEdge(size_t edgeNode1, size_t edgeNode2, size_t midNodeId);
    void lawsonFlip(const std::vector<size_t>& newTriangleIds);

    MeshData2D& meshData_;
    PeriodicMeshData2D* periodicData_ = nullptr;
    MeshQueries2D queries_;
    std::unique_ptr<MeshMutator2D> mutator_;
    std::unique_ptr<ElementGeometry2D> geometry_;
};

} // namespace Meshing
