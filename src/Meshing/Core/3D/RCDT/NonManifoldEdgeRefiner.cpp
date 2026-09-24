#include "Meshing/Core/3D/RCDT/NonManifoldEdgeRefiner.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/RCDTPointInserter.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"
#include "Meshing/Core/3D/RCDT/SurfaceProjector.h"
#include "Meshing/Data/3D/MeshData3D.h"
#include "Meshing/Data/CurveSegmentManager.h"

namespace Meshing
{

NonManifoldEdgeRefiner::NonManifoldEdgeRefiner(const MeshingContext3D& context,
                                               const RestrictedTriangulation& restrictedTriangulation,
                                               double minimumEdgeLength) :
    context_(&context),
    restrictedTriangulation_(&restrictedTriangulation),
    minimumEdgeLength_(minimumEdgeLength)
{
}

bool NonManifoldEdgeRefiner::refineNext(RCDTPointInserter& pointInserter)
{
    const auto& meshData = context_->getMeshData();
    const auto& curveSegmentManager = meshData.getCurveSegmentManager();
    const auto* geometry = context_->getGeometry();
    if (!geometry)
        return false;

    const auto& nodePositionMap = pointInserter.getNodePositionMap();
    const auto defects = restrictedTriangulation_->findNonManifoldEdges(meshData);

    for (const auto& defect : defects)
    {
        if (unrefinableNonManifoldEdges_.count(defect.edge))
            continue;

        const auto firstEndpoint = nodePositionMap.find(defect.edge.nodeIds[0]);
        const auto secondEndpoint = nodePositionMap.find(defect.edge.nodeIds[1]);
        if (firstEndpoint == nodePositionMap.end() || secondEndpoint == nodePositionMap.end())
        {
            unrefinableNonManifoldEdges_.insert(defect.edge);
            continue;
        }

        // Size floor -- but NOT the cutoff that actually stops this
        // priority, despite looking like the one the other priorities use.
        // Instrumented over OPE-184's 65-defect repro: of 107 refusals this
        // test fired ZERO times. The operative cutoff is twice this value,
        // enforced by RCDTPointInserter's proximity guard: the repair point
        // is the defect edge's own midpoint, so its distance to that edge's
        // OWN endpoints is length/2 by construction, which falls below the
        // floor for any edge shorter than 2 * minimumEdgeLength_. 50 of the
        // 56 proximity refusals were blocked by the defect edge's own
        // endpoint that way.
        //
        // Kept rather than deleted: "never fires" was measured on one model
        // at one pinned floor, and it is still the honest statement of this
        // priority's own intent. Exempting the proximity guard to let repair
        // proceed below 2x is separately DISPROVEN -- OPE-184 direction 2
        // measured 65 -> 1242 defects -- so do not treat this comment as an
        // invitation to lower the real cutoff.
        const double length = (firstEndpoint->second - secondEndpoint->second).norm();
        if (length <= minimumEdgeLength_)
        {
            unrefinableNonManifoldEdges_.insert(defect.edge);
            continue;
        }

        // If the endpoints are joined by a curve segment, split it rather than
        // project onto one of the surfaces: the defects traced in OPE-170/171
        // sat on a crease, and a projected point lands near the crease but not
        // on it, which was measured to grow the defect count. If
        // trySplitSegment() declines, the defect is marked unrefinable rather
        // than falling back to that projection.
        if (const auto segmentId = curveSegmentManager.findSegmentId(defect.edge.nodeIds[0], defect.edge.nodeIds[1]))
        {
            if (pointInserter.trySplitSegment(*segmentId))
                return true;
            unrefinableNonManifoldEdges_.insert(defect.edge);
            continue;
        }

        const Geometry3D::ISurface3D* surface = geometry->getSurface(defect.surfaceId);
        if (!surface)
        {
            unrefinableNonManifoldEdges_.insert(defect.edge);
            continue;
        }

        const Point3D midpoint = 0.5 * (firstEndpoint->second + secondEndpoint->second);
        const auto projectedMidpoint = SurfaceProjector::projectToSurface(midpoint, *surface);
        if (!projectedMidpoint)
        {
            unrefinableNonManifoldEdges_.insert(defect.edge);
            continue;
        }
        const Point3D& insertionPoint = *projectedMidpoint;

        if (pointInserter.tryInsert(insertionPoint, {defect.surfaceId}))
            return true;
        unrefinableNonManifoldEdges_.insert(defect.edge);
    }

    return false;
}

} // namespace Meshing
