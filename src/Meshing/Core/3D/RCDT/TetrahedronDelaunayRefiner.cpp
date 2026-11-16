#include "Meshing/Core/3D/RCDT/TetrahedronDelaunayRefiner.h"

#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/General/RegularConflictRegion.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedronElement.h"
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

constexpr ElementId INVALID_ID = SIZE_MAX;

std::optional<double> squaredRadiusEdgeRatio(const TetrahedronElement& tetrahedron, const MeshData3D& meshData);

} // namespace

TetrahedronDelaunayRefiner::TetrahedronDelaunayRefiner(MeshingContext3D& context,
                                                       const Topology3D::Topology3D& topology,
                                                       const SurfaceMesh3DQualitySettings& settings,
                                                       double minimumEdgeLength) :
    context_(&context),
    surfaceRefiner_(context, topology, settings, minimumEdgeLength),
    ratioBound_(settings.tetrahedronCircumradiusToShortestEdgeRatio)
{
}

void TetrahedronDelaunayRefiner::refine()
{
    surfaceRefiner_.refine();
    while (!surfaceRefiner_.hasReachedInsertionCap() && refineRound())
    {
    }
    if (surfaceRefiner_.hasReachedInsertionCap())
        spdlog::warn("TetrahedronDelaunayRefiner: reached the insertion cap");
    surfaceRefiner_.writeSnapshot("refined");
    spdlog::info("TetrahedronDelaunayRefiner: {} interior insertions, {} encroached facets refined instead, "
                 "{} tetrahedra left unrefined",
                 insertionCount_, encroachmentCount_, unrefinable_.size());
}

/// One pass over the bad tetrahedra inside the domain, worst first, as
/// labelled at its start. Tetrahedra an earlier insertion in the pass
/// destroyed are skipped; their replacements are judged next pass.
bool TetrahedronDelaunayRefiner::refineRound()
{
    auto& meshData = context_->getMeshData();
    auto ambient = AmbientTetrahedronClassifier::classify(meshData, surfaceRefiner_.getRestrictedFaces());

    double squaredBound = ratioBound_ * ratioBound_;
    std::vector<std::pair<double, std::size_t>> badTetrahedra;
    for (auto& [elementId, element] : meshData.getElements())
    {
        auto tetrahedron = dynamic_cast<const TetrahedronElement*>(element.get());
        if (!tetrahedron || ambient.contains(elementId) || unrefinable_.contains(elementId))
            continue;
        if (auto ratio = squaredRadiusEdgeRatio(*tetrahedron, meshData); ratio && *ratio > squaredBound)
            badTetrahedra.emplace_back(*ratio, elementId);
    }
    std::sort(badTetrahedra.begin(), badTetrahedra.end(), std::greater<>());

    bool inserted = false;
    for (auto [ratio, elementId] : badTetrahedra)
    {
        if (surfaceRefiner_.hasReachedInsertionCap())
            break;
        auto tetrahedron = dynamic_cast<const TetrahedronElement*>(meshData.getElement(elementId));
        if (tetrahedron && refineTetrahedron(elementId, *tetrahedron))
            inserted = true;
    }
    return inserted;
}

bool TetrahedronDelaunayRefiner::refineTetrahedron(ElementId elementId, const TetrahedronElement& tetrahedron)
{
    auto& meshData = context_->getMeshData();
    auto orthocenter = ElementGeometry3D(meshData).computeOrthocenter(tetrahedron);
    if (!orthocenter)
    {
        unrefinable_.insert(elementId);
        return false;
    }

    auto conflicting =
        RegularConflictRegion::find({elementId, INVALID_ID}, *orthocenter, meshData, surfaceRefiner_.getConnectivity());
    if (conflicting.empty())
    {
        unrefinable_.insert(elementId);
        return false;
    }

    if (auto encroached = surfaceRefiner_.findEncroachedFacet(*orthocenter, conflicting))
    {
        if (!surfaceRefiner_.refineFacet(*encroached))
        {
            unrefinable_.insert(elementId);
            return false;
        }
        ++encroachmentCount_;
        surfaceRefiner_.refineQueued();
        return true;
    }

    if (RegularConflictRegion::isHiddenOrDuplicate(*orthocenter, conflicting, meshData))
    {
        unrefinable_.insert(elementId);
        return false;
    }

    surfaceRefiner_.insert(*orthocenter, std::move(conflicting), {});
    ++insertionCount_;
    surfaceRefiner_.refineQueued();
    return true;
}

namespace
{

/// Squared weighted circumradius over squared shortest edge, CGAL's
/// Cell_radius_edge_criterion. nullopt for a tetrahedron without an
/// orthocentre or with a non-positive weighted radius, which no insertion
/// inside it can improve.
std::optional<double> squaredRadiusEdgeRatio(const TetrahedronElement& tetrahedron, const MeshData3D& meshData)
{
    auto orthocenter = ElementGeometry3D(meshData).computeOrthocenter(tetrahedron);
    if (!orthocenter)
        return std::nullopt;
    auto& nodeIds = tetrahedron.getNodeIds();
    auto node0 = meshData.getNode(nodeIds[0]);
    double squaredRadius = (*orthocenter - node0->getCoordinates()).squaredNorm() - node0->getWeight();
    double squaredShortestEdge = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = i + 1; j < 4; ++j)
            squaredShortestEdge =
                std::min(squaredShortestEdge,
                         (meshData.getNode(nodeIds[i])->getCoordinates() - meshData.getNode(nodeIds[j])->getCoordinates())
                             .squaredNorm());
    if (squaredRadius <= 0.0 || squaredShortestEdge <= 0.0)
        return std::nullopt;
    return squaredRadius / squaredShortestEdge;
}

} // namespace

} // namespace Meshing
