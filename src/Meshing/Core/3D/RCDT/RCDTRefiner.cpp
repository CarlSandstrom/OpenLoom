#include "Meshing/Core/3D/RCDT/RCDTRefiner.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/Base/MeshConnectivity.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "spdlog/spdlog.h"

#include <cmath>
#include <optional>
#include <unordered_set>

namespace Meshing
{

RCDTRefiner::RCDTRefiner(MeshingContext3D& context,
                         RestrictedTriangulation& restrictedTriangulation,
                         const SurfaceMesh3DQualitySettings& settings,
                         double minimumEdgeLength,
                         const RCDTTetQualityController* tetQualityController) :
    context_(&context),
    restrictedTriangulation_(&restrictedTriangulation),
    settings_(settings),
    minimumEdgeLength_(minimumEdgeLength),
    pointInserter_(context, restrictedTriangulation, minimumEdgeLength),
    nonManifoldEdgeRefiner_(context, restrictedTriangulation, minimumEdgeLength)
{
    if (tetQualityController)
        tetrahedronQualityRefiner_.emplace(context,
                                           restrictedTriangulation,
                                           *tetQualityController,
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

    const size_t maxIterations = settings_.maxRefinementIterations;
    while (iteration < maxIterations)
    {
        if (!refineStep()) break;
        exportMesh3D(context_->getMeshData(), "rcdt_refinement_step", iteration);
        ++iteration;
    }

    if (iteration >= maxIterations)
        spdlog::warn("RCDTRefiner: reached iteration cap ({})", maxIterations);

    spdlog::info("RCDTRefiner: done after {} iterations — {} nodes",
                 iteration,
                 context_->getMeshData().getNodeCount());
}

bool RCDTRefiner::refineStep()
{
    pointInserter_.beginStep();

    if (pointInserter_.splitEncroachedSegment())
        return true;

    // ---- Priority 2: bad restricted triangles ----

    const auto& meshData = context_->getMeshData();
    const auto* geometry = context_->getGeometry();
    if (!geometry)
        return false;

    const MeshConnectivity connectivity(meshData);
    const auto badTriangles = restrictedTriangulation_->getBadTriangles();

    if (badTriangles.empty())
        return refineRemainingPriorities();

    for (const auto& bad : badTriangles)
    {
        if (unrefinableTriangles_.count(bad.face))
            continue;

        // Size floor (CGAL/Boissonnat-Oudot style): a triangle at or below
        // minimumEdgeLength_ is left as-is even if still quality-bad. Checked
        // first because it needs no surface work.
        if (bad.shortestEdge <= minimumEdgeLength_)
        {
            unrefinableTriangles_.insert(bad.face);
            continue;
        }

        const Geometry3D::ISurface3D* surface = geometry->getSurface(bad.surfaceId);
        if (!surface)
        {
            unrefinableTriangles_.insert(bad.face);
            continue;
        }

        // Prefer the restricted Voronoi vertex, where the face's dual edge
        // crosses the surface. When there is none -- usually because the
        // face's classification is stale and its current dual edge no longer
        // crosses the surface -- fall back to projecting the circumcenter; the
        // proximity guard in RCDTPointInserter::tryInsert() rejects a fallback
        // point that lands on an existing vertex. Dropping the fallback was
        // measured worse: most fallback points are usable. Computed for this
        // one triangle rather than for every bad triangle, since it bisects
        // with OCC calls.
        std::optional<Point3D> projectedOpt =
            restrictedTriangulation_->computeInsertionPoint(bad.face, meshData, connectivity, *surface);
        if (!projectedOpt)
            projectedOpt = surfaceProjector_.projectToSurface(bad.circumcircleCenter, *surface);
        if (!projectedOpt)
        {
            unrefinableTriangles_.insert(bad.face);
            continue;
        }

        const Point3D& projected = *projectedOpt;

        if (pointInserter_.tryInsert(projected, {bad.surfaceId}))
            return true;
        unrefinableTriangles_.insert(bad.face);
    }

    return refineRemainingPriorities();
}

bool RCDTRefiner::refineRemainingPriorities()
{
    if (tetrahedronQualityRefiner_ && tetrahedronQualityRefiner_->refineNext(pointInserter_))
        return true;
    return nonManifoldEdgeRefiner_.refineNext(pointInserter_);
}

} // namespace Meshing
