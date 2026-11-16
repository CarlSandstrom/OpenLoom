#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Common/Exceptions/MeshException.h"
#include "Meshing/Connectivity/EdgeKey.h"
#include "Meshing/Core/3D/General/RobustPredicates3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Meshing/Data/3D/Node3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"
#include "spdlog/spdlog.h"
#include <algorithm>
#include <limits>
#include <set>
#include <unordered_set>

namespace Meshing
{

namespace
{

// A coplanar cavity boundary face that growCavityThroughCoplanarFaces()
// declined to grow through (see its comment): the two tetrahedra that used
// to share it -- one on the conflicting (removed) side, one on the kept
// (neighbor) side -- need a local 3-way split around the new vertex instead
// of a plain fan. Captured before either tetrahedron is removed, since
// findOppositeVertex() needs both to still exist.
struct CoplanarFaceSplit
{
    std::array<size_t, 3> face;
    size_t apex0;
    size_t neighborTetrahedronId;
    size_t apex1;
};

// The cavity boundary faces a plain vertex fan covers, and the coplanar
// faces that need a split instead.
struct CavityBoundaryPartition
{
    std::vector<std::array<size_t, 3>> normalFaces;
    std::vector<CoplanarFaceSplit> splits;
};

CavityBoundaryPartition partitionCavityBoundary(const Point3D& point,
                                                const std::vector<size_t>& conflicting,
                                                const MeshData3D& meshData,
                                                const MeshQueries3D& queries);
bool isCoplanarWithFace(const Point3D& vertex, const std::array<size_t, 3>& face, const MeshData3D& meshData);
int eulerCharacteristic(const std::vector<std::array<size_t, 3>>& boundary);

} // namespace

MeshOperations3D::MeshOperations3D(MeshData3D& meshData) :
    meshData_(meshData),
    queries_(meshData),
    mutator_(std::make_unique<MeshMutator3D>(meshData))
{
}

std::array<size_t, 4> MeshOperations3D::createBoundingTetrahedron(const std::vector<Point3D>& points)
{
    if (points.empty())
    {
        OPENLOOM_THROW_CODE(OpenLoom::MeshException,
                            OpenLoom::MeshException::ErrorCode::INVALID_OPERATION,
                            "createBoundingTetrahedron called with empty point list");
    }

    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();
    double maxX = std::numeric_limits<double>::lowest();
    double maxY = std::numeric_limits<double>::lowest();
    double maxZ = std::numeric_limits<double>::lowest();

    for (const auto& point : points)
    {
        minX = std::min(minX, point.x());
        minY = std::min(minY, point.y());
        minZ = std::min(minZ, point.z());
        maxX = std::max(maxX, point.x());
        maxY = std::max(maxY, point.y());
        maxZ = std::max(maxZ, point.z());
    }

    double extentX = maxX - minX;
    double extentY = maxY - minY;
    double extentZ = maxZ - minZ;
    // At least 1.0 so a degenerate (flat or single-point) set still gets a usable margin
    double maxDimension = std::max({extentX, extentY, extentZ, 1.0});
    double margin = maxDimension * 10.0;

    // One corner below the box's minimum corner, three reaching far out along each axis
    Point3D corner0(minX - margin, minY - margin, minZ - margin);
    Point3D corner1(maxX + 3.0 * margin, minY - margin, minZ - margin);
    Point3D corner2(minX - margin, maxY + 3.0 * margin, minZ - margin);
    Point3D corner3(minX - margin, minY - margin, maxZ + 3.0 * margin);

    std::array<size_t, 4> nodeIds = {mutator_->addNode(corner0),
                                     mutator_->addNode(corner1),
                                     mutator_->addNode(corner2),
                                     mutator_->addNode(corner3)};

    mutator_->addElement(std::make_unique<TetrahedronElement>(nodeIds));
    mutator_->setBoundingNodeIds(nodeIds);

    spdlog::debug("MeshOperations3D::createBoundingTetrahedron: Created with nodes ({}, {}, {}, {})",
                  nodeIds[0], nodeIds[1], nodeIds[2], nodeIds[3]);

    return nodeIds;
}

size_t MeshOperations3D::insertVertexBowyerWatson(const Point3D& point,
                                                  const std::vector<std::string>& geometryIds,
                                                  double weight)
{
    return insertVertexBowyerWatson(point, queries_.findConflictingTetrahedra(point, weight), geometryIds, weight);
}

size_t MeshOperations3D::insertVertexBowyerWatson(const Point3D& point,
                                                  std::vector<size_t> conflicting,
                                                  const std::vector<std::string>& geometryIds,
                                                  double weight)
{
    if (conflicting.empty())
    {
        size_t nodeId = mutator_->addNode(point, weight);
        spdlog::warn("MeshOperations3D::insertVertexBowyerWatson: No conflicting tetrahedra found (node {}, "
                     "point ({}, {}, {}), weight {})",
                     nodeId, point.x(), point.y(), point.z(), weight);
        return nodeId;
    }

    conflicting = growCavityThroughCoplanarFaces(point, std::move(conflicting));
    auto partition = partitionCavityBoundary(point, conflicting, meshData_, queries_);

    // Remove conflicting tetrahedra, plus each split's neighbor tetrahedron
    // (captured by partitionCavityBoundary() before removal, since it isn't in `conflicting`).
    for (size_t tetrahedronId : conflicting)
    {
        mutator_->removeElement(tetrahedronId);
    }
    for (const auto& split : partition.splits)
    {
        mutator_->removeElement(split.neighborTetrahedronId);
    }

    auto nodeId = geometryIds.empty() ? mutator_->addNode(point, weight) : mutator_->addBoundaryNode(point, geometryIds, weight);

    retriangulate(nodeId, partition.normalFaces);
    for (const auto& split : partition.splits)
    {
        splitCoplanarBoundaryFace(nodeId, split.face, split.apex0, split.apex1);
    }

    return nodeId;
}

std::vector<size_t> MeshOperations3D::growCavityThroughCoplanarFaces(
    const Point3D& point, std::vector<size_t> conflicting) const
{
    std::unordered_set<size_t> conflictingSet(conflicting.begin(), conflicting.end());

    bool grew = true;
    while (grew)
    {
        grew = false;

        for (const auto& face : queries_.findCavityBoundary(conflicting))
        {
            if (!isCoplanarWithFace(point, face, meshData_))
                continue;

            auto touching = queries_.findTetrahedraWithFace(face[0], face[1], face[2]);
            auto neighbor = std::find_if(touching.begin(), touching.end(),
                                         [&conflictingSet](size_t tetrahedronId)
                                         { return !conflictingSet.contains(tetrahedronId); });

            // Either there's no neighbor to extend into (a genuine domain
            // boundary) or both sides are already conflicting -- either way,
            // nothing more to do for this face here; insertVertexBowyerWatson()
            // handles whatever's left on the boundary afterward.
            if (neighbor == touching.end())
                continue;

            // Tentatively grow through this face and check whether the
            // result is still a topological sphere before committing
            // (OPE-173): growing indiscriminately can wrap the cavity into a
            // non-simply-connected shape (e.g. a solid-torus-shaped cavity
            // from wrapping all the way around an axis), which
            // retriangulate()'s single-point vertex fan cannot validly
            // cover. If committing would break that, leave this face
            // unresolved -- insertVertexBowyerWatson() falls back to a local
            // coplanar-face split for it instead.
            conflicting.push_back(*neighbor);
            if (eulerCharacteristic(queries_.findCavityBoundary(conflicting)) != 2)
            {
                conflicting.pop_back();
                continue;
            }

            conflictingSet.insert(*neighbor);
            grew = true;
        }
    }

    return conflicting;
}

void MeshOperations3D::addOrientedTetrahedron(const std::array<size_t, 3>& face, size_t apexNodeId)
{
    auto node0 = meshData_.getNode(face[0]);
    auto node1 = meshData_.getNode(face[1]);
    auto node2 = meshData_.getNode(face[2]);
    auto apexNode = meshData_.getNode(apexNodeId);
    if (!node0 || !node1 || !node2 || !apexNode)
        return;

    auto point0 = node0->getCoordinates();
    auto point1 = node1->getCoordinates();
    auto point2 = node2->getCoordinates();
    auto apexPoint = apexNode->getCoordinates();

    Point3D faceNormal = (point1 - point0).cross(point2 - point0);
    double signedVolume = faceNormal.dot(apexPoint - point0);

    // signedVolume is the signed volume of (point0, point1, point2, apexPoint) in that vertex
    // order (by the scalar triple product's cyclic invariance, faceNormal
    // . (apexPoint - point0) == (point1 - point0) . ((point2 - point0) x (apexPoint - point0))). Store the node
    // IDs in that same (face..., apex) order so a >= 0 signedVolume here
    // really does mean a positive-orientation TetrahedronElement under the
    // usual (face..., apex) sign convention -- storing the apex first
    // instead (an odd permutation) would silently negate it.
    auto nodeIds = signedVolume >= 0.0 ? std::array<size_t, 4>{face[0], face[1], face[2], apexNodeId} : std::array<size_t, 4>{face[0], face[2], face[1], apexNodeId};

    mutator_->addElement(std::make_unique<TetrahedronElement>(nodeIds));
}

void MeshOperations3D::retriangulate(size_t vertexNodeId,
                                     const std::vector<std::array<size_t, 3>>& boundary)
{
    for (const auto& face : boundary)
    {
        addOrientedTetrahedron(face, vertexNodeId);
    }
}

void MeshOperations3D::splitCoplanarBoundaryFace(size_t vertexNodeId,
                                                 const std::array<size_t, 3>& face,
                                                 size_t apex0,
                                                 size_t apex1)
{
    // face and vertexNodeId are coplanar (that's why this face reached here
    // instead of retriangulate()): fanning a single tetrahedron onto face
    // from either apex would be zero-volume. Instead split face into 3
    // sub-triangles around vertexNodeId and fan each to BOTH apexes -- the
    // standard "pyramid subdivision" of the two tetrahedra (face, apex0) and
    // (face, apex1) that used to share this face, now sharing vertexNodeId
    // as an interior point of their common base instead.
    std::array<std::array<size_t, 3>, 3> subFaces = {{
        {face[0], face[1], vertexNodeId},
        {face[1], face[2], vertexNodeId},
        {face[2], face[0], vertexNodeId},
    }};
    for (const auto& subFace : subFaces)
    {
        addOrientedTetrahedron(subFace, apex0);
        addOrientedTetrahedron(subFace, apex1);
    }
}

namespace
{

// Whether vertex lies exactly in the plane through the face's three nodes -- i.e.
// whether the tetrahedron (vertex, face...) would have zero volume. Exact, not
// tolerance-based (see RobustPredicates3D). A face with a missing node counts
// as not coplanar.
bool isCoplanarWithFace(const Point3D& vertex, const std::array<size_t, 3>& face, const MeshData3D& meshData)
{
    auto node0 = meshData.getNode(face[0]);
    auto node1 = meshData.getNode(face[1]);
    auto node2 = meshData.getNode(face[2]);
    if (!node0 || !node1 || !node2)
        return false;
    return RobustPredicates3D::orientationSign(node0->getCoordinates(), node1->getCoordinates(),
                                               node2->getCoordinates(), vertex) == 0;
}

// Euler characteristic of the closed surface formed by a set of boundary
// triangles -- V - E + F. A topological sphere (the shape retriangulate()'s
// single-point vertex fan requires -- every boundary face must be "visible"
// via a straight line from the new vertex that stays inside the cavity) is
// always 2. growCavityThroughCoplanarFaces() uses this to detect when
// growing the cavity through one more coplanar face would produce something
// else -- e.g. a solid-torus-shaped cavity (Euler characteristic 0) from
// wrapping all the way around an axis, or a non-manifold boundary (an odd
// characteristic; see OPE-173) -- and declines that growth instead of
// producing a cavity the vertex fan can't validly cover.
int eulerCharacteristic(const std::vector<std::array<size_t, 3>>& boundary)
{
    std::set<size_t> vertices;
    std::set<EdgeKey> edges;
    for (const auto& face : boundary)
    {
        vertices.insert(face.begin(), face.end());
        for (size_t i = 0; i < 3; ++i)
            edges.insert(EdgeKey(face[i], face[(i + 1) % 3]));
    }
    return static_cast<int>(vertices.size()) - static_cast<int>(edges.size()) + static_cast<int>(boundary.size());
}

// Partitions the cavity boundary: growCavityThroughCoplanarFaces() only grows
// through a coplanar face when doing so keeps the cavity a topological
// sphere (OPE-173); any coplanar face it declined to grow through is
// still on the boundary here and needs a local 3-way split around the
// new vertex instead of retriangulate()'s single-point fan, which would
// otherwise create a zero-volume tetrahedron.
CavityBoundaryPartition partitionCavityBoundary(const Point3D& point,
                                                const std::vector<size_t>& conflicting,
                                                const MeshData3D& meshData,
                                                const MeshQueries3D& queries)
{
    std::unordered_set<size_t> conflictingSet(conflicting.begin(), conflicting.end());
    CavityBoundaryPartition partition;
    for (const auto& face : queries.findCavityBoundary(conflicting))
    {
        if (!isCoplanarWithFace(point, face, meshData))
        {
            partition.normalFaces.push_back(face);
            continue;
        }

        auto touching = queries.findTetrahedraWithFace(face[0], face[1], face[2]);
        auto conflictingSide = std::find_if(
            touching.begin(), touching.end(), [&](size_t tetrahedronId)
            { return conflictingSet.contains(tetrahedronId); });
        auto neighbor = std::find_if(
            touching.begin(), touching.end(), [&](size_t tetrahedronId)
            { return !conflictingSet.contains(tetrahedronId); });

        if (conflictingSide == touching.end() || neighbor == touching.end())
        {
            // No usable neighbor to split against -- a genuine domain
            // boundary (nothing on the other side) or some other unusual
            // configuration. Fall back to a plain fan rather than fail the
            // insertion.
            partition.normalFaces.push_back(face);
            continue;
        }

        auto apex0 = queries.findOppositeVertex(*conflictingSide, face[0], face[1], face[2]);
        auto apex1 = queries.findOppositeVertex(*neighbor, face[0], face[1], face[2]);
        if (!apex0 || !apex1)
        {
            partition.normalFaces.push_back(face);
            continue;
        }

        partition.splits.push_back({face, *apex0, *neighbor, *apex1});
    }
    return partition;
}

} // namespace

} // namespace Meshing
