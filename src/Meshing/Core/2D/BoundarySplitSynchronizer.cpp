#include "BoundarySplitSynchronizer.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include "Meshing/Core/2D/MeshOperations2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Data/2D/MeshData2D.h"

namespace Meshing
{

BoundarySplitSynchronizer::BoundarySplitSynchronizer(MeshingContext2D& context,
                                                     TwinManager& twinManager) :
    context_(&context),
    twinManager_(&twinManager)
{
}

void BoundarySplitSynchronizer::operator()(size_t n1, size_t n2, size_t mid)
{
    auto twin = twinManager_->getTwin(TwinManager::NO_SURFACE, n1, n2);
    if (!twin)
        return;

    auto [twinSurface, t1, t2] = *twin;

    const auto& manager = context_->getMeshData().getCurveSegmentManager();
    auto segmentIdOpt = manager.findSegmentId(t1, t2);
    if (!segmentIdOpt)
        return;

    const CurveSegment twinSegment = manager.getSegment(*segmentIdOpt);

    const auto* twinEdge = context_->getGeometry().getEdge(twinSegment.edgeId);
    if (!twinEdge)
        return;

    auto twinMid = context_->getOperations().splitConstrainedSegment(twinSegment, *twinEdge);
    if (!twinMid)
        return;

    // Use TwinManager direction (t1, t2) so endpoint correspondence is preserved
    twinManager_->recordSplit(TwinManager::NO_SURFACE, n1, n2, mid,
                              TwinManager::NO_SURFACE, t1, t2, *twinMid);
}

} // namespace Meshing
