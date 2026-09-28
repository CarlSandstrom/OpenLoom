#include "Meshing/Core/3D/RCDT/SurfaceDelaunayRefiner.h"

#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/RegularConflictRegion.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <array>
#include <cstdint>
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

    refineQueued();

    if (hasReachedInsertionCap())
        spdlog::warn("SurfaceDelaunayRefiner: reached the insertion cap ({})", settings_.maxRefinementIterations);
    spdlog::info("SurfaceDelaunayRefiner: {} insertions, {} restricted facets, {} still bad, {} dropped",
                 insertionCount_, facets_.size(), queue_.size(), droppedFaces_.size());
}

void SurfaceDelaunayRefiner::refineQueued()
{
    while (!hasReachedInsertionCap() && refineWorst())
    {
    }
}

bool SurfaceDelaunayRefiner::hasReachedInsertionCap() const
{
    return insertionCount_ >= settings_.maxRefinementIterations;
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
    while (!queue_.empty())
        if (refineFacet(queue_.begin()->second))
            return true;
    return false;
}

bool SurfaceDelaunayRefiner::refineFacet(const FaceKey& face)
{
    auto& meshData = context_->getMeshData();
    const RestrictedFacet facet = facets_.at(face);

    // CGAL's refusals: the point must conflict with a tetrahedron beside the
    // facet, or inserting it would not remove the facet; and it must not be
    // hidden by, or coincide with, an existing vertex.
    const auto& [elementId1, elementId2] = connectivity_->getFaceElements(face);
    auto conflicting = RegularConflictRegion::find({elementId1, elementId2}, facet.surfaceCenter, meshData, *connectivity_);
    if (conflicting.empty() || RegularConflictRegion::isHiddenOrDuplicate(facet.surfaceCenter, conflicting, meshData))
    {
        drop(face);
        return false;
    }
    insert(facet.surfaceCenter, std::move(conflicting), {facet.surfaceId});
    return true;
}

void SurfaceDelaunayRefiner::drop(const FaceKey& face)
{
    const auto badness = badness_.find(face);
    if (badness != badness_.end())
    {
        queue_.erase({badness->second, face});
        badness_.erase(badness);
    }
    droppedFaces_.insert(face);
}

void SurfaceDelaunayRefiner::insert(const Point3D& point,
                                    std::vector<size_t> conflictRegion,
                                    std::vector<std::string> geometryIds)
{
    auto& meshData = context_->getMeshData();

    std::vector<FaceKey> cavityFaces;
    for (const size_t elementId : conflictRegion)
        if (const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId)))
            for (const auto& cavityFace : facesOf(*tetrahedron))
                cavityFaces.push_back(cavityFace);

    const size_t newNodeId =
        context_->getOperations().insertVertexBowyerWatson(point, std::move(conflictRegion), std::move(geometryIds));
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
}

std::optional<FaceKey> SurfaceDelaunayRefiner::findEncroachedFacet(const Point3D& point,
                                                                   const std::vector<size_t>& conflictRegion) const
{
    const auto& meshData = context_->getMeshData();
    for (const size_t elementId : conflictRegion)
    {
        const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId));
        if (!tetrahedron)
            continue;
        for (const auto& face : facesOf(*tetrahedron))
        {
            const auto facet = facets_.find(face);
            if (facet == facets_.end())
                continue;
            // The surface Delaunay ball: centred on the dual line, so at equal
            // power to all three weighted vertices.
            const Node3D* vertex = meshData.getNode(face.nodeIds[0]);
            const Point3D& center = facet->second.surfaceCenter;
            const double squaredRadius = (center - vertex->getCoordinates()).squaredNorm() - vertex->getWeight();
            if ((point - center).squaredNorm() < squaredRadius)
                return face;
        }
    }
    return std::nullopt;
}

} // namespace Meshing
