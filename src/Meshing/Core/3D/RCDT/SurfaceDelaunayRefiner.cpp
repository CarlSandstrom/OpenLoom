#include "Meshing/Core/3D/RCDT/SurfaceDelaunayRefiner.h"

#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/General/RegularPredicates3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <deque>
#include <unordered_set>
#include <vector>

namespace Meshing
{

namespace
{

constexpr size_t INVALID_ID = SIZE_MAX;

std::unordered_set<std::string> surfaceIdsOf(const Topology3D::Topology3D& topology)
{
    const auto surfaceIds = topology.getAllSurfaceIds();
    return {surfaceIds.begin(), surfaceIds.end()};
}

std::vector<FaceKey> facesOf(const TetrahedralElement& tetrahedron)
{
    std::vector<FaceKey> faces;
    for (const auto& faceNodes : tetrahedron.getFaces())
        faces.emplace_back(faceNodes);
    return faces;
}

bool conflicts(const TetrahedralElement& tetrahedron, const Point3D& point, const MeshData3D& meshData)
{
    const auto& ids = tetrahedron.getNodeIds();
    const Node3D* n0 = meshData.getNode(ids[0]);
    const Node3D* n1 = meshData.getNode(ids[1]);
    const Node3D* n2 = meshData.getNode(ids[2]);
    const Node3D* n3 = meshData.getNode(ids[3]);
    return RegularPredicates3D::insidePointOrthosphere(n0->getCoordinates(), n0->getWeight(), n1->getCoordinates(),
                                                       n1->getWeight(), n2->getCoordinates(), n2->getWeight(),
                                                       n3->getCoordinates(), n3->getWeight(), point, 0.0);
}

/// The tetrahedra in conflict with point, grown from seeds across shared
/// faces. In a regular triangulation that region is connected, so this finds
/// the same set as scanning every tetrahedron, at the cost of the region
/// alone. Empty when no seed conflicts -- CGAL's "facet not in its conflict
/// zone".
std::vector<size_t> conflictRegion(const std::array<size_t, 2>& seeds,
                                   const Point3D& point,
                                   const MeshData3D& meshData,
                                   const MeshConnectivity& connectivity)
{
    std::vector<size_t> region;
    std::unordered_set<size_t> visited;
    std::deque<size_t> pending;
    for (const size_t seed : seeds)
        if (seed != INVALID_ID && visited.insert(seed).second)
            pending.push_back(seed);

    while (!pending.empty())
    {
        const size_t elementId = pending.front();
        pending.pop_front();
        const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId));
        if (!tetrahedron || !conflicts(*tetrahedron, point, meshData))
            continue;
        region.push_back(elementId);
        for (const auto& face : facesOf(*tetrahedron))
        {
            const auto& [first, second] = connectivity.getFaceElements(face);
            for (const size_t neighbour : {first, second})
                if (neighbour != INVALID_ID && visited.insert(neighbour).second)
                    pending.push_back(neighbour);
        }
    }
    return region;
}

/// Whether point would be hidden by, or coincide with, a vertex of the
/// tetrahedra it conflicts with: power distance |point - v|^2 - w_v <= 0. A
/// hidden point is not a vertex of the regular triangulation, so inserting it
/// breaks the triangulation; a coincident one duplicates a node. CGAL's
/// insert refuses both.
bool isHiddenOrDuplicate(const Point3D& point, const std::vector<size_t>& region, const MeshData3D& meshData)
{
    for (const size_t elementId : region)
    {
        const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId));
        for (const size_t nodeId : tetrahedron->getNodeIds())
        {
            const Node3D* node = meshData.getNode(nodeId);
            if ((node->getCoordinates() - point).squaredNorm() <= node->getWeight())
                return true;
        }
    }
    return false;
}

} // namespace

SurfaceDelaunayRefiner::SurfaceDelaunayRefiner(MeshingContext3D& context,
                                               const Topology3D::Topology3D& topology,
                                               const SurfaceMesh3DQualitySettings& settings,
                                               double minimumEdgeLength) :
    context_(&context),
    settings_(settings),
    restriction_(*context.getGeometry(), topology, minimumEdgeLength),
    criteria_(settings, surfaceIdsOf(topology))
{
}

SurfaceDelaunayRefiner::~SurfaceDelaunayRefiner() = default;

void SurfaceDelaunayRefiner::refine()
{
    connectivity_ = std::make_unique<MeshConnectivity>(context_->getMeshData());
    classifyAllFaces();
    spdlog::info("SurfaceDelaunayRefiner: {} restricted facets, {} bad", facets_.size(), queue_.size());

    while (insertionCount_ < settings_.maxRefinementIterations && refineWorst())
    {
    }

    if (insertionCount_ >= settings_.maxRefinementIterations)
        spdlog::warn("SurfaceDelaunayRefiner: reached the insertion cap ({})", settings_.maxRefinementIterations);
    spdlog::info("SurfaceDelaunayRefiner: {} insertions, {} restricted facets, {} still bad, {} dropped",
                 insertionCount_, facets_.size(), queue_.size(), droppedFaces_.size());
}

RestrictedFaceMap SurfaceDelaunayRefiner::getRestrictedFaces() const
{
    RestrictedFaceMap faces;
    for (const auto& [face, facet] : facets_)
        faces.emplace(face, facet.surfaceId);
    return faces;
}

void SurfaceDelaunayRefiner::classifyAllFaces()
{
    std::unordered_set<FaceKey, FaceKeyHash> seen;
    for (const auto& [elementId, element] : context_->getMeshData().getElements())
    {
        const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tetrahedron)
            continue;
        for (const auto& face : facesOf(*tetrahedron))
            if (seen.insert(face).second)
                reclassify(face);
    }
}

void SurfaceDelaunayRefiner::forget(const FaceKey& face)
{
    const auto badness = badness_.find(face);
    if (badness != badness_.end())
    {
        queue_.erase({badness->second, face});
        badness_.erase(badness);
    }
    facets_.erase(face);
    droppedFaces_.erase(face);
}

void SurfaceDelaunayRefiner::reclassify(const FaceKey& face)
{
    forget(face);
    const auto facet = restriction_.restrict(face, context_->getMeshData(), *connectivity_);
    if (!facet)
        return;
    facets_.emplace(face, *facet);
    if (const auto badness = criteria_.findBadness(face, *facet, context_->getMeshData()))
    {
        badness_.emplace(face, *badness);
        queue_.emplace(*badness, face);
    }
}

bool SurfaceDelaunayRefiner::refineWorst()
{
    auto& meshData = context_->getMeshData();
    auto& operations = context_->getOperations();

    while (!queue_.empty())
    {
        const FaceKey face = queue_.begin()->second;
        const RestrictedFacet facet = facets_.at(face);

        // CGAL's refusals: the point must conflict with a tetrahedron beside
        // the facet, or inserting it would not remove the facet; and it must
        // not be hidden by, or coincide with, an existing vertex.
        const auto& [elementId1, elementId2] = connectivity_->getFaceElements(face);
        auto conflicting = conflictRegion({elementId1, elementId2}, facet.surfaceCenter, meshData, *connectivity_);
        if (conflicting.empty() || isHiddenOrDuplicate(facet.surfaceCenter, conflicting, meshData))
        {
            const auto badness = badness_.find(face);
            queue_.erase({badness->second, face});
            badness_.erase(badness);
            droppedFaces_.insert(face);
            continue;
        }

        std::vector<FaceKey> cavityFaces;
        for (const size_t elementId : conflicting)
            if (const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId)))
                for (const auto& cavityFace : facesOf(*tetrahedron))
                    cavityFaces.push_back(cavityFace);

        const size_t newNodeId =
            operations.insertVertexBowyerWatson(facet.surfaceCenter, std::move(conflicting), {facet.surfaceId});
        ++insertionCount_;

        connectivity_ = std::make_unique<MeshConnectivity>(meshData);
        for (const auto& cavityFace : cavityFaces)
        {
            if (connectivity_->getFaceElements(cavityFace).first == INVALID_ID)
                forget(cavityFace);
            else
                reclassify(cavityFace);
        }
        for (const size_t elementId : connectivity_->getNodeElements(newNodeId))
            if (const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId)))
                for (const auto& newFace : facesOf(*tetrahedron))
                    reclassify(newFace);
        return true;
    }
    return false;
}

} // namespace Meshing
