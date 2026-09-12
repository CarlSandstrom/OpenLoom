#include "Meshing/Core/3D/RCDT/RCDTRefiner.h"

#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/SurfaceMesh3DQualitySettings.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "spdlog/spdlog.h"

#include <optional>

namespace Meshing
{

RCDTRefiner::RCDTRefiner(MeshingContext3D& context,
                         RestrictedTriangulation& restrictedTriangulation,
                         const SurfaceMesh3DQualitySettings& settings,
                         double minimumEdgeLength,
                         const RCDTTetQualityController* tetrahedronQualityController) :
    context_(&context),
    maximumRefinementIterations_(settings.maxRefinementIterations),
    minimumEdgeLength_(minimumEdgeLength),
    pointInserter_(context, restrictedTriangulation, minimumEdgeLength),
    restrictedTriangleRefiner_(context, restrictedTriangulation, minimumEdgeLength),
    nonManifoldEdgeRefiner_(context, restrictedTriangulation, minimumEdgeLength)
{
    if (tetrahedronQualityController)
        tetrahedronQualityRefiner_.emplace(context,
                                           restrictedTriangulation,
                                           *tetrahedronQualityController,
                                           settings.tetCircumradiusToShortestEdgeRatio,
                                           minimumEdgeLength);
}

void RCDTRefiner::refine()
{
    const auto& meshData = context_->getMeshData();
    spdlog::info("RCDTRefiner: starting refinement — {} nodes, {} segments",
                 meshData.getNodeCount(),
                 meshData.getCurveSegmentManager().size());

    spdlog::info("RCDTRefiner: minimum edge length floor = {}", minimumEdgeLength_);

    pointInserter_.seedEncroachedSegments();

    size_t iteration = 0;
    exportMesh3D(context_->getMeshData(), "rcdt_refinement_step", iteration);
    ++iteration;

    while (iteration < maximumRefinementIterations_)
    {
        if (!refineStep()) break;
        exportMesh3D(context_->getMeshData(), "rcdt_refinement_step", iteration);
        ++iteration;
    }

    if (iteration >= maximumRefinementIterations_)
        spdlog::warn("RCDTRefiner: reached iteration cap ({})", maximumRefinementIterations_);

    spdlog::info("RCDTRefiner: done after {} iterations — {} nodes",
                 iteration,
                 context_->getMeshData().getNodeCount());
}

bool RCDTRefiner::refineStep()
{
    pointInserter_.beginStep();

    if (pointInserter_.splitEncroachedSegment())
        return true;

    // Priorities 2-4 all place their points on the geometry.
    if (!context_->getGeometry())
        return false;

    if (restrictedTriangleRefiner_.refineNext(pointInserter_))
        return true;

    if (tetrahedronQualityRefiner_ && tetrahedronQualityRefiner_->refineNext(pointInserter_))
        return true;

    return nonManifoldEdgeRefiner_.refineNext(pointInserter_);
}

} // namespace Meshing
