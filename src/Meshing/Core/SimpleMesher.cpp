#include "SimpleMesher.h"
#include "MeshingContext.h"

#include "../Data/MeshOperations.h"
#include "../Data/TetrahedralElement.h"
#include "../Operations/ScopedTransaction.h"

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

    // Seed 4 nodes forming a unit tetrahedron
    size_t n0 = ops.addNode({0.0, 0.0, 0.0});
    size_t n1 = ops.addNode({1.0, 0.0, 0.0});
    size_t n2 = ops.addNode({0.0, 1.0, 0.0});
    siwwze_t n3 = ops.addNode({0.0, 0.0, 1.0});

    // Create a single tetrahedral element
    std::array<size_t, 4> tetNodes{n0, n1, n2, n3};
    auto tet = std::make_unique<TetrahedralElement>(tetNodes);
    (void)ops.addElement(std::move(tet));

    // Build connectivity
    context.rebuildConnectivity();

    // Commit on success
    txn.commit();
}

} // namespace Meshing
