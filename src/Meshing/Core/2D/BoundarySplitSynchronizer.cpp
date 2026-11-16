#include "Meshing/Core/2D/BoundarySplitSynchronizer.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Meshing/Core/2D/MeshOperations2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/CurveSegmentManager.h"

namespace Meshing
{

BoundarySplitSynchronizer::BoundarySplitSynchronizer(MeshingContext2D& context,
                                                     TwinManager& twinManager) :
    context_(&context),
    twinManager_(&twinManager)
{
}

void BoundarySplitSynchronizer::operator()(size_t node0, size_t node1, size_t midpointNode)
{
    auto twin = twinManager_->getTwin(TwinManager::NO_SURFACE, node0, node1);
    if (!twin)
        return;

    auto [twinSurface, twinNode0, twinNode1] = *twin;

    auto& manager = context_->getMeshData().getCurveSegmentManager();
    auto optionalSegmentId = manager.findSegmentId(twinNode0, twinNode1);
    if (!optionalSegmentId)
        return;

    auto twinSegment = manager.getSegment(*optionalSegmentId);

    auto twinEdge = context_->getGeometry().getEdge(twinSegment.edgeId);
    if (!twinEdge)
        return;

    auto twinMidpointNode =
        context_->getOperations().splitConstrainedSegment(twinSegment, *twinEdge);
    if (!twinMidpointNode)
        return;

    // Use TwinManager direction (twinNode0, twinNode1) so endpoint correspondence is preserved
    twinManager_->recordSplit(TwinManager::NO_SURFACE, node0, node1, midpointNode,
                              TwinManager::NO_SURFACE, twinNode0, twinNode1, *twinMidpointNode);
}

} // namespace Meshing
