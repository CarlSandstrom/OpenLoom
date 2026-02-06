#include "ConstrainedDelaunay3D.h"
#include "Delaunay3D.h"
#include "MeshingContext3D.h"
#include "MeshOperations3D.h"
#include "MeshQueries3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/MeshMutator3D.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

ConstrainedDelaunay3D::ConstrainedDelaunay3D(
    MeshingContext3D& context,
    const DiscretizationResult3D& discretization) :
    discretization_(discretization),
    context_(&context),
    meshData3D_(&context.getMeshData()),
    meshOperations_(&context.getOperations())
{
}

ConstrainedDelaunay3D::~ConstrainedDelaunay3D() = default;

ConstrainedDelaunay3D::ConstrainedDelaunay3D(ConstrainedDelaunay3D&&) noexcept = default;
ConstrainedDelaunay3D& ConstrainedDelaunay3D::operator=(ConstrainedDelaunay3D&&) noexcept = default;

void ConstrainedDelaunay3D::tetrahedralize()
{
    spdlog::info("ConstrainedDelaunay3D: Starting tetrahedralization with {} points",
                 discretization_.points.size());

    // Step 1: Create initial Delaunay tetrahedralization (unconstrained)
    Delaunay3D delaunay(
        discretization_.points,
        meshData3D_,
        discretization_.edgeParameters,
        discretization_.geometryIds);
    delaunay.triangulate();

    // Store the point-to-node mapping for later use
    pointIndexToNodeIdMap_ = delaunay.getPointIndexToNodeIdMap();

    spdlog::info("ConstrainedDelaunay3D: Initial tetrahedralization complete - {} nodes, {} tetrahedra",
                 meshData3D_->getNodeCount(), meshData3D_->getElementCount());

    // Step 2: Extract and store constrained subsegments
    extractAndStoreSubsegments();

    // Step 3: Create facet triangulations
    createFacetTriangulations();

    // Step 4: Store subfacets from facet triangulations
    storeSubfacets();

    spdlog::info("ConstrainedDelaunay3D: Constraint setup complete - {} subsegments, {} subfacets",
                 meshData3D_->getConstrainedSubsegments().size(),
                 meshData3D_->getConstrainedSubfacets().size());
}

void ConstrainedDelaunay3D::extractAndStoreSubsegments()
{
    const auto* topology = context_->getTopology();
    if (!topology)
    {
        spdlog::warn("ConstrainedDelaunay3D: No topology available, skipping subsegment extraction");
        return;
    }

    // Extract subsegments using MeshQueries3D
    MeshQueries3D queries(*meshData3D_);
    auto subsegments = queries.extractConstrainedSubsegments(
        *topology,
        discretization_.cornerIdToPointIndexMap,
        pointIndexToNodeIdMap_,
        discretization_.edgeIdToPointIndicesMap);

    // Store subsegments in MeshData3D
    auto& mutator = context_->getMutator();
    for (const auto& subsegment : subsegments)
    {
        mutator.addConstrainedSubsegment(subsegment);
    }

    spdlog::debug("ConstrainedDelaunay3D: Extracted and stored {} subsegments", subsegments.size());
}

void ConstrainedDelaunay3D::createFacetTriangulations()
{
    const auto* geometry = context_->getGeometry();
    const auto* topology = context_->getTopology();

    if (!geometry || !topology)
    {
        spdlog::warn("ConstrainedDelaunay3D: No geometry/topology available, skipping facet triangulations");
        return;
    }

    // Create the facet triangulation manager
    facetManager_ = std::make_unique<FacetTriangulationManager>(*geometry, *topology);

    // Initialize from the discretization
    facetManager_->initializeFromDiscretization(
        discretization_,
        pointIndexToNodeIdMap_,
        *meshData3D_);

    spdlog::debug("ConstrainedDelaunay3D: Created {} facet triangulations", facetManager_->size());
}

void ConstrainedDelaunay3D::storeSubfacets()
{
    if (!facetManager_)
    {
        spdlog::warn("ConstrainedDelaunay3D: No facet manager available, skipping subfacet storage");
        return;
    }

    // Get all subfacets from the facet triangulations
    auto subfacets = facetManager_->getAllSubfacets();

    // Store subfacets in MeshData3D
    auto& mutator = context_->getMutator();
    for (const auto& subfacet : subfacets)
    {
        mutator.addConstrainedSubfacet(subfacet);
    }

    spdlog::debug("ConstrainedDelaunay3D: Stored {} subfacets", subfacets.size());
}

} // namespace Meshing
