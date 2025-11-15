#include "SimpleMesher.h"
#include "../Data/MeshOperations.h"
#include "../Data/TetrahedralElement.h"
#include "../Operations/ScopedTransaction.h"
#include "Geometry/Base/GeometryCollection.h"
#include "MeshingContext.h"
#include "Topology/Topology.h"

#include <array>
#include <memory>

namespace Meshing
{

void SimpleMesher::generate(MeshingContext& context)
{
    // Start with a clean mesh for this simple example
    context.clearMesh();

    auto& ops = context.getOperations();

    // Begin transaction to ensure rollback on any failure
    Meshing::ScopedTransaction txn(&ops);
    std::vector<size_t> cornerNodeIds;
    for (auto cornerId : context.getTopology().getAllCornerIds())
    {
        auto geometryCorner = context.getGeometry().getCorner(cornerId);
        auto point = geometryCorner->getPoint();
        cornerNodeIds.push_back(ops.addNode(point));
    }

    // Build connectivity
    context.rebuildConnectivity();

    // Commit on success
    txn.commit();
}

} // namespace Meshing
