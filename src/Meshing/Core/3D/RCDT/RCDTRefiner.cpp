#include "Meshing/Core/3D/RCDT/RCDTRefiner.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/ElementGeometry3D.h"
#include "Meshing/Core/3D/General/MeshDebugUtils3D.h"
#include "Meshing/Core/3D/General/MeshOperations3D.h"
#include "Meshing/Core/3D/General/MeshQueries3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/AmbientTetrahedronClassifier.h"
#include "Meshing/Core/3D/RCDT/RCDTTetQualityController.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/3D/TetrahedralElement.h"
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
    tetQualityController_(tetQualityController),
    minimumEdgeLength_(minimumEdgeLength),
    pointInserter_(context, restrictedTriangulation, minimumEdgeLength),
    nonManifoldEdgeRefiner_(context, restrictedTriangulation, minimumEdgeLength)
{
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
    if (refineBadTetrahedra())
        return true;
    return nonManifoldEdgeRefiner_.refineNext(pointInserter_);
}

bool RCDTRefiner::refineBadTetrahedra()
{
    if (!tetQualityController_)
        return false;

    const auto& meshData = context_->getMeshData();

    // AmbientTetrahedronRemover only runs after refinement, so the mesh still
    // contains ambient tetrahedra: those touching the bounding tetrahedron's
    // corners, and those filling holes in the domain. Neither is output, and
    // both are skinny by nature, so refining them would spend iterations on
    // meaningless circumcenters. AmbientTetrahedronClassifier finds both.
    const auto ambientTetIds = AmbientTetrahedronClassifier::classify(meshData, *restrictedTriangulation_);

    const auto skinnyTetIds =
        context_->getOperations().getQueries().findSkinnyTetrahedra(settings_.tetCircumradiusToShortestEdgeRatio);

    const ElementGeometry3D elementGeometry(meshData);

    for (const size_t tetId : skinnyTetIds)
    {
        if (unrefinableTetrahedra_.count(tetId))
            continue;

        const auto* element = meshData.getElement(tetId);
        const auto* tet = dynamic_cast<const TetrahedralElement*>(element);
        if (!tet)
            continue;

        if (ambientTetIds.contains(tetId))
            continue;

        // Size floor: a degenerate tetrahedron is left unrefined.
        if (tetQualityController_->isTetrahedronTooSmall(*tet))
        {
            unrefinableTetrahedra_.insert(tetId);
            continue;
        }

        const auto circumsphere = elementGeometry.computeCircumscribingSphere(*tet);
        if (!circumsphere)
        {
            unrefinableTetrahedra_.insert(tetId);
            continue;
        }

        // As a tetrahedron flattens, its circumcenter recedes toward infinity
        // (real geometry, not numerical error), so inserting it can land far
        // outside the mesh -- radius 126 was measured on a unit box -- and
        // refinement never converges. The bound uses minimumEdgeLength_, the
        // mesh's own scale, rather than this tetrahedron's possibly tiny size.
        // This is the sliver limitation in the class doc.
        constexpr double MAX_CIRCUMRADIUS_TO_MIN_EDGE_LENGTH_RATIO = 100.0;
        if (circumsphere->radius > MAX_CIRCUMRADIUS_TO_MIN_EDGE_LENGTH_RATIO * minimumEdgeLength_)
        {
            unrefinableTetrahedra_.insert(tetId);
            continue;
        }

        const Point3D& circumcenter = circumsphere->center;

        // Interior point: no geometryIds, matching insertVertexBowyerWatson's
        // convention for a non-boundary node.
        if (pointInserter_.tryInsert(circumcenter, {}))
            return true;
        unrefinableTetrahedra_.insert(tetId);
    }

    return false;
}

} // namespace Meshing
