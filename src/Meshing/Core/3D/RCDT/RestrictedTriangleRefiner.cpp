#include "Meshing/Core/3D/RCDT/RestrictedTriangleRefiner.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/RCDTPointInserter.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/Base/MeshConnectivity.h"

#include <optional>

namespace Meshing
{

RestrictedTriangleRefiner::RestrictedTriangleRefiner(const MeshingContext3D& context,
                                                     const RestrictedTriangulation& restrictedTriangulation,
                                                     double minimumEdgeLength) :
    context_(&context),
    restrictedTriangulation_(&restrictedTriangulation),
    minimumEdgeLength_(minimumEdgeLength)
{
}

bool RestrictedTriangleRefiner::refineNext(RCDTPointInserter& pointInserter)
{
    const auto& meshData = context_->getMeshData();
    const auto* geometry = context_->getGeometry();
    if (!geometry)
        return false;

    const MeshConnectivity connectivity(meshData);
    const auto badTriangles = restrictedTriangulation_->getBadTriangles();

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

        if (pointInserter.tryInsert(projected, {bad.surfaceId}))
            return true;
        unrefinableTriangles_.insert(bad.face);
    }

    return false;
}

} // namespace Meshing
