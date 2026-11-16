#include "Meshing/Core/3D/RCDT/SurfaceDelaunayRefiner.h"

#include "Export/VtkGrid.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshSnapshotWriter.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/General/RegularConflictRegion.h"
#include "Meshing/Core/3D/RCDT/SnapshotGridBuilder.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace Meshing
{

namespace
{
std::unordered_set<std::string> surfaceIdsOf(const Topology3D::Topology3D& topology);
std::vector<FaceKey> facesOf(const TetrahedronElement& tetrahedron);
std::vector<FaceKey> facesOfElements(const MeshData3D& meshData, const std::vector<ElementId>& elementIds);
void removeFromQueue(const FaceKey& face,
                     std::unordered_map<FaceKey, SurfaceFacetCriteria::Badness, FaceKeyHash>& badness,
                     std::set<std::pair<SurfaceFacetCriteria::Badness, FaceKey>>& queue);
} // namespace

SurfaceDelaunayRefiner::SurfaceDelaunayRefiner(MeshingContext3D& context,
                                               const Topology3D::Topology3D& topology,
                                               const SurfaceMesh3DQualitySettings& settings,
                                               double minimumEdgeLength) :
    context_(&context),
    topology_(&topology),
    settings_(settings),
    restriction_(*context.getGeometry(), topology, minimumEdgeLength),
    criteria_(settings, surfaceIdsOf(topology))
{
}

SurfaceDelaunayRefiner::~SurfaceDelaunayRefiner() = default;

void SurfaceDelaunayRefiner::refine()
{
    classifyAllFaces();
    spdlog::info("SurfaceDelaunayRefiner: {} restricted facets, {} bad", facets_.size(), queue_.size());
    writeSnapshot("seeded");
    if (MeshSnapshotWriter::isEnabled())
    {
        MeshSnapshotWriter::write("balls", SnapshotGridBuilder::fromProtectingBalls(context_->getMeshData()));
    }

    refineQueued();

    if (hasReachedInsertionCap())
    {
        spdlog::warn("SurfaceDelaunayRefiner: reached the insertion cap ({})", settings_.maxRefinementIterations);
    }
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
    for (auto& [face, facet] : facets_)
        faces.emplace(face, facet.surfaceId);
    return faces;
}

void SurfaceDelaunayRefiner::classifyAllFaces()
{
    std::unordered_set<FaceKey, FaceKeyHash> seen;
    for (auto& [elementId, element] : context_->getMeshData().getElements())
    {
        auto tetrahedron = dynamic_cast<const TetrahedronElement*>(element.get());
        if (!tetrahedron) continue;

        for (auto& face : facesOf(*tetrahedron))
        {
            if (seen.insert(face).second) reclassify(face);
        }
    }
}

void SurfaceDelaunayRefiner::forget(const FaceKey& face)
{
    removeFromQueue(face, badness_, queue_);
    facets_.erase(face);
    droppedFaces_.erase(face);
}

void SurfaceDelaunayRefiner::reclassify(const FaceKey& face)
{
    forget(face);
    auto facet = restriction_.restrict(face, context_->getMeshData(), context_->getConnectivity());
    if (!facet) return;

    facets_.emplace(face, *facet);
    if (auto badness = criteria_.findBadness(face, *facet, context_->getMeshData()))
    {
        badness_.emplace(face, *badness);
        queue_.emplace(*badness, face);
    }
}

bool SurfaceDelaunayRefiner::refineWorst()
{
    while (!queue_.empty())
    {
        if (refineFacet(queue_.begin()->second)) return true;
    }
    return false;
}

bool SurfaceDelaunayRefiner::refineFacet(const FaceKey& face)
{
    auto& meshData = context_->getMeshData();
    auto facet = facets_.at(face);

    // The point must conflict with a tetrahedron beside the facet, or inserting
    // it would not remove the facet; and it must not be hidden by, or coincide
    // with, an existing vertex.
    auto& connectivity = context_->getConnectivity();
    auto [elementId1, elementId2] = connectivity.getFaceElements(face);
    auto conflicting = RegularConflictRegion::find({elementId1, elementId2}, facet.surfaceCenter, meshData, connectivity);
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
    removeFromQueue(face, badness_, queue_);
    droppedFaces_.insert(face);
}

void SurfaceDelaunayRefiner::insert(const Point3D& point,
                                    std::vector<ElementId> conflictRegion,
                                    std::vector<std::string> geometryIds)
{
    auto& meshData = context_->getMeshData();

    auto cavityFaces = facesOfElements(meshData, conflictRegion);

    auto newNodeId =
        context_->getOperations().insertVertexBowyerWatson(point, std::move(conflictRegion), std::move(geometryIds));
    ++insertionCount_;

    auto& connectivity = context_->getConnectivity();
    for (auto& cavityFace : cavityFaces)
    {
        if (connectivity.getFaceElements(cavityFace).first == ElementId::INVALID)
            forget(cavityFace);
        else
            reclassify(cavityFace);
    }
    for (auto& newFace : facesOfElements(meshData, connectivity.getNodeElements(newNodeId)))
    {
        reclassify(newFace);
    }

    if (MeshSnapshotWriter::isInsertionDue(insertionCount_)) writeSnapshot("insertion");
}

void SurfaceDelaunayRefiner::writeSnapshot(const std::string& label) const
{
    if (!MeshSnapshotWriter::isEnabled()) return;

    MeshSnapshotWriter::write(label,
                              SnapshotGridBuilder::fromRestrictedFacets(
                                  context_->getMeshData(), *topology_, facets_, badness_, droppedFaces_));
}

std::optional<FaceKey> SurfaceDelaunayRefiner::findEncroachedFacet(const Point3D& point,
                                                                   const std::vector<ElementId>& conflictRegion) const
{
    auto& meshData = context_->getMeshData();
    for (auto elementId : conflictRegion)
    {
        auto tetrahedron = dynamic_cast<const TetrahedronElement*>(meshData.getElement(elementId));
        if (!tetrahedron) continue;

        for (auto& face : facesOf(*tetrahedron))
        {
            auto facet = facets_.find(face);
            if (facet == facets_.end()) continue;

            // The surface Delaunay ball: centred on the dual line, so at equal
            // power to all three weighted vertices.
            auto vertex = meshData.getNode(face.nodeIds[0]);
            auto& center = facet->second.surfaceCenter;
            double squaredRadius = (center - vertex->getCoordinates()).squaredNorm() - vertex->getWeight();
            if ((point - center).squaredNorm() < squaredRadius) return face;
        }
    }
    return std::nullopt;
}

namespace
{

std::unordered_set<std::string> surfaceIdsOf(const Topology3D::Topology3D& topology)
{
    auto surfaceIds = topology.getAllSurfaceIds();
    return {surfaceIds.begin(), surfaceIds.end()};
}

std::vector<FaceKey> facesOf(const TetrahedronElement& tetrahedron)
{
    std::vector<FaceKey> faces;
    for (auto& faceNodes : tetrahedron.getFaces())
        faces.emplace_back(faceNodes);
    return faces;
}

std::vector<FaceKey> facesOfElements(const MeshData3D& meshData, const std::vector<ElementId>& elementIds)
{
    std::vector<FaceKey> faces;
    for (auto elementId : elementIds)
    {
        auto tetrahedron = dynamic_cast<const TetrahedronElement*>(meshData.getElement(elementId));
        if (!tetrahedron) continue;

        for (auto& face : facesOf(*tetrahedron))
            faces.push_back(face);
    }
    return faces;
}

void removeFromQueue(const FaceKey& face,
                     std::unordered_map<FaceKey, SurfaceFacetCriteria::Badness, FaceKeyHash>& badness,
                     std::set<std::pair<SurfaceFacetCriteria::Badness, FaceKey>>& queue)
{
    auto entry = badness.find(face);
    if (entry == badness.end()) return;

    queue.erase({entry->second, face});
    badness.erase(entry);
}

} // namespace

} // namespace Meshing
