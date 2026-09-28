#include "Meshing/Core/3D/RCDT/TetrahedronDelaunayRefiner.h"

#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier.h"
#include "Meshing/Core/3D/RCDT/RegularConflictRegion.h"
#include "Meshing/Core/3D/RCDT/SurfaceDelaunayRefiner.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
#include "spdlog/spdlog.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace Meshing
{

namespace
{

constexpr std::size_t INVALID_ID = SIZE_MAX;

/// Squared weighted circumradius over squared shortest edge, CGAL's
/// Cell_radius_edge_criterion. nullopt for a tetrahedron without an
/// orthocentre or with a non-positive weighted radius, which no insertion
/// inside it can improve.
std::optional<double> squaredRadiusEdgeRatio(const TetrahedralElement& tetrahedron, const MeshData3D& meshData)
{
    const auto orthocenter = ElementGeometry3D(meshData).computeOrthocenter(tetrahedron);
    if (!orthocenter)
        return std::nullopt;
    const auto& ids = tetrahedron.getNodeIds();
    const Node3D* first = meshData.getNode(ids[0]);
    const double squaredRadius = (*orthocenter - first->getCoordinates()).squaredNorm() - first->getWeight();
    double squaredShortestEdge = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = i + 1; j < 4; ++j)
            squaredShortestEdge =
                std::min(squaredShortestEdge,
                         (meshData.getNode(ids[i])->getCoordinates() - meshData.getNode(ids[j])->getCoordinates())
                             .squaredNorm());
    if (squaredRadius <= 0.0 || squaredShortestEdge <= 0.0)
        return std::nullopt;
    return squaredRadius / squaredShortestEdge;
}

} // namespace

TetrahedronDelaunayRefiner::TetrahedronDelaunayRefiner(MeshingContext3D& context,
                                                       SurfaceDelaunayRefiner& surfaceRefiner,
                                                       const SurfaceMesh3DQualitySettings& settings) :
    context_(&context),
    surfaceRefiner_(&surfaceRefiner),
    ratioBound_(settings.tetCircumradiusToShortestEdgeRatio)
{
}

void TetrahedronDelaunayRefiner::refine()
{
    surfaceRefiner_->refine();
    while (!surfaceRefiner_->hasReachedInsertionCap() && refineRound())
    {
    }
    if (surfaceRefiner_->hasReachedInsertionCap())
        spdlog::warn("TetrahedronDelaunayRefiner: reached the insertion cap");
    spdlog::info("TetrahedronDelaunayRefiner: {} interior insertions, {} encroached facets refined instead, "
                 "{} tetrahedra left unrefined",
                 insertionCount_, encroachmentCount_, unrefinable_.size());
}

/// One pass over the bad tetrahedra inside the domain, worst first, as
/// labelled at its start. Tetrahedra an earlier insertion in the pass
/// destroyed are skipped; their replacements are judged next pass.
bool TetrahedronDelaunayRefiner::refineRound()
{
    const auto& meshData = context_->getMeshData();
    const auto ambient = AmbientTetrahedronClassifier::classify(meshData, surfaceRefiner_->getRestrictedFaces());

    const double squaredBound = ratioBound_ * ratioBound_;
    std::vector<std::pair<double, std::size_t>> bad;
    for (const auto& [elementId, element] : meshData.getElements())
    {
        const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(element.get());
        if (!tetrahedron || ambient.contains(elementId) || unrefinable_.contains(elementId))
            continue;
        if (const auto ratio = squaredRadiusEdgeRatio(*tetrahedron, meshData); ratio && *ratio > squaredBound)
            bad.emplace_back(*ratio, elementId);
    }
    std::sort(bad.begin(), bad.end(), std::greater<>());

    bool inserted = false;
    for (const auto& [ratio, elementId] : bad)
    {
        if (surfaceRefiner_->hasReachedInsertionCap())
            break;
        const auto* tetrahedron = dynamic_cast<const TetrahedralElement*>(meshData.getElement(elementId));
        if (tetrahedron && refineTetrahedron(elementId, *tetrahedron))
            inserted = true;
    }
    return inserted;
}

bool TetrahedronDelaunayRefiner::refineTetrahedron(std::size_t elementId, const TetrahedralElement& tetrahedron)
{
    const auto& meshData = context_->getMeshData();
    const auto orthocenter = ElementGeometry3D(meshData).computeOrthocenter(tetrahedron);
    if (!orthocenter)
    {
        unrefinable_.insert(elementId);
        return false;
    }

    auto conflicting =
        RegularConflictRegion::find({elementId, INVALID_ID}, *orthocenter, meshData, surfaceRefiner_->getConnectivity());
    if (conflicting.empty())
    {
        unrefinable_.insert(elementId);
        return false;
    }

    if (const auto encroached = surfaceRefiner_->findEncroachedFacet(*orthocenter, conflicting))
    {
        const bool refined = surfaceRefiner_->refineFacet(*encroached);
        if (!refined)
        {
            unrefinable_.insert(elementId);
            return false;
        }
        ++encroachmentCount_;
        surfaceRefiner_->refineQueued();
        return true;
    }

    if (RegularConflictRegion::isHiddenOrDuplicate(*orthocenter, conflicting, meshData))
    {
        unrefinable_.insert(elementId);
        return false;
    }

    surfaceRefiner_->insert(*orthocenter, std::move(conflicting), {});
    ++insertionCount_;
    surfaceRefiner_->refineQueued();
    return true;
}

} // namespace Meshing
