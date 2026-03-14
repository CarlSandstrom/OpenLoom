#include "BoundarySplitSynchronizer.h"
#include "Geometry/2D/Base/IEdge2D.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Core/2D/MeshOperations2D.h"
#include "Meshing/Core/2D/MeshQueries2D.h"
#include "Meshing/Core/2D/MeshingContext2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include <algorithm>

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

    // The stored ConstrainedSegment2D may use either endpoint order, so search
    // for both {t1,t2} and {t2,t1}.
    const auto& segs = context_->getMeshData().getConstrainedSegments();
    auto it = std::find_if(segs.begin(), segs.end(),
                           [&](const ConstrainedSegment2D& s) {
                               return (s.nodeId1 == t1 && s.nodeId2 == t2) ||
                                      (s.nodeId1 == t2 && s.nodeId2 == t1);
                           });
    if (it == segs.end())
        return;

    const ConstrainedSegment2D twinSeg = *it;

    auto twinEdgeId = context_->getOperations().getQueries().findCommonGeometryId(
        twinSeg.nodeId1, twinSeg.nodeId2);
    if (!twinEdgeId)
        return;

    const auto* twinEdge = context_->getGeometry().getEdge(*twinEdgeId);
    if (!twinEdge)
        return;

    auto twinMid = context_->getOperations().splitConstrainedSegment(twinSeg, *twinEdge);
    if (!twinMid)
        return;

    // Use TwinManager direction (t1, t2) so endpoint correspondence is preserved
    twinManager_->recordSplit(TwinManager::NO_SURFACE, n1, n2, mid,
                              TwinManager::NO_SURFACE, t1, t2, *twinMid);
}

} // namespace Meshing
