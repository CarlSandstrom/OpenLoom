#include "Meshing/Core/3D/RCDT/NonManifoldEdgeRefiner.h"

#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/ISurface3D.h"
#include "Meshing/Core/3D/General/MeshingContext3D.h"
#include "Meshing/Core/3D/RCDT/RCDTPointInserter.h"
#include "Meshing/Core/3D/RCDT/RestrictedTriangulation.h"
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

        const auto it1 = nodePositionMap.find(defect.edge.nodeIds[0]);
        const auto it2 = nodePositionMap.find(defect.edge.nodeIds[1]);
        if (it1 == nodePositionMap.end() || it2 == nodePositionMap.end())
        {
            unrefinableNonManifoldEdges_.insert(defect.edge);
            continue;
        }

        // Size floor, same reasoning as the other priorities.
        const double length = (it1->second - it2->second).norm();
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

        const Point3D midpoint = 0.5 * (it1->second + it2->second);
        const auto projectedOpt = surfaceProjector_.projectToSurface(midpoint, *surface);
        if (!projectedOpt)
        {
            unrefinableNonManifoldEdges_.insert(defect.edge);
            continue;
        }
        const Point3D& projected = *projectedOpt;

        if (pointInserter.tryInsert(projected, {defect.surfaceId}))
            return true;
        unrefinableNonManifoldEdges_.insert(defect.edge);
    }

    return false;
}

} // namespace Meshing
