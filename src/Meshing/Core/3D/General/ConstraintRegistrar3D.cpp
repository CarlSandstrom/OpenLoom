#include "Meshing/Core/3D/General/ConstraintRegistrar3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/General/MeshQueries3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

ConstraintRegistrar3D::ConstraintRegistrar3D(
    MeshingContext3D& context,
    const DiscretizationResult3D& discretization) :
    context_(&context),
    discretization_(&discretization)
{
}

ConstraintRegistrar3D::~ConstraintRegistrar3D() = default;

ConstraintRegistrar3D::ConstraintRegistrar3D(ConstraintRegistrar3D&&) noexcept = default;
ConstraintRegistrar3D& ConstraintRegistrar3D::operator=(ConstraintRegistrar3D&&) noexcept = default;

void ConstraintRegistrar3D::registerConstraints(const std::map<size_t, size_t>& pointIndexToNodeIdMap)
{
    pointIndexToNodeIdMap_ = pointIndexToNodeIdMap;

    spdlog::debug("ConstraintRegistrar3D: Registering constraints from discretization");

    registerSubsegments();
    registerSubfacets();

    spdlog::info("ConstraintRegistrar3D: Registered {} subsegments, {} subfacets",
                 context_->getMeshData().getConstrainedSubsegmentCount(),
                 context_->getMeshData().getConstrainedSubfacetCount());
}

void ConstraintRegistrar3D::registerSubsegments()
{
    const auto* topology = context_->getTopology();
    if (!topology)
    {
        spdlog::warn("ConstraintRegistrar3D: No topology available, skipping subsegment registration");
        return;
    }

    MeshQueries3D queries(context_->getMeshData());
    auto subsegments = queries.extractConstrainedSubsegments(
        *topology,
        discretization_->cornerIdToPointIndexMap,
        pointIndexToNodeIdMap_,
        discretization_->edgeIdToPointIndicesMap);

    auto& mutator = context_->getMutator();
    for (const auto& subsegment : subsegments)
    {
        mutator.addConstrainedSubsegment(subsegment);
    }

    spdlog::debug("ConstraintRegistrar3D: Registered {} subsegments", subsegments.size());
}

void ConstraintRegistrar3D::registerSubfacets()
{
    const auto* geometry = context_->getGeometry();
    const auto* topology = context_->getTopology();

    if (!geometry || !topology)
    {
        spdlog::warn("ConstraintRegistrar3D: No geometry/topology available, skipping subfacet registration");
        return;
    }

    facetManager_ = std::make_unique<FacetTriangulationManager>(*geometry, *topology);

    facetManager_->initializeFromDiscretization(
        *discretization_,
        pointIndexToNodeIdMap_,
        context_->getMeshData());

    auto subfacets = facetManager_->getAllSubfacets();

    auto& mutator = context_->getMutator();
    for (const auto& subfacet : subfacets)
    {
        mutator.addConstrainedSubfacet(subfacet);
    }

    spdlog::debug("ConstraintRegistrar3D: Registered {} subfacets from {} facet triangulations",
                  subfacets.size(), facetManager_->size());
}

std::unique_ptr<FacetTriangulationManager> ConstraintRegistrar3D::releaseFacetManager()
{
    return std::move(facetManager_);
}

} // namespace Meshing
