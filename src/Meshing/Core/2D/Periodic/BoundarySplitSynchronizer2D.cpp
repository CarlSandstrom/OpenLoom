#include "Meshing/Core/2D/Periodic/BoundarySplitSynchronizer2D.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Meshing/Core/2D/General/MeshOperations2D.h"
#include "Meshing/Core/2D/General/MeshingContext2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/CurveSegmentManager.h"

namespace Meshing
{

BoundarySplitSynchronizer2D::BoundarySplitSynchronizer2D(MeshingContext2D& context,
                                                         TwinManager2D& twinManager) :
    context_(&context),
    twinManager_(&twinManager)
{
}

void BoundarySplitSynchronizer2D::operator()(NodeId node0, NodeId node1, NodeId midpointNode)
{
    auto twin = twinManager_->getTwin(TwinManager2D::NO_SURFACE, node0, node1);
    if (!twin)
        return;

    auto [twinSurface, twinNode0, twinNode1] = *twin;

    auto& manager = context_->getMeshData().getCurveSegmentManager();
    auto optionalSegmentId = manager.findSegmentId(twinNode0, twinNode1);
    if (!optionalSegmentId)
        return;

    auto twinSegment = manager.getSegment(*optionalSegmentId);

    auto twinEdge = context_->getGeometry().getEdge(twinSegment.edgeId);
    auto twinMidpointNode =
        context_->getOperations().splitConstrainedSegment(twinSegment, *twinEdge);
    if (!twinMidpointNode)
        return;

    // Use TwinManager2D direction (twinNode0, twinNode1) so endpoint correspondence is preserved
    twinManager_->recordSplit(TwinManager2D::NO_SURFACE, node0, node1, midpointNode,
                              TwinManager2D::NO_SURFACE, twinNode0, twinNode1, *twinMidpointNode);
}

} // namespace Meshing
