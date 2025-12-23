#include "ConstrainedMesher.h"
#include "ConstrainedDelaunay3D.h"
#include "Geometry/Base/GeometryCollection3D.h"
#include "MeshingContext3D.h"
#include "Topology/Topology3D.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

void ConstrainedMesher::generate(MeshingContext3D& context)
{
    SPDLOG_INFO("ConstrainedMesher: Starting mesh generation");
    SPDLOG_INFO("  Edge samples: {}", samplesPerEdge_);
    SPDLOG_INFO("  Surface samples: {}x{}", samplesPerSurface_, samplesPerSurface_);

    // Clear any existing mesh
    context.clearMesh();

    // Create constrained Delaunay mesher
    ConstrainedDelaunay3D mesher(context);

    // Generate constrained mesh respecting topology (edges AND faces)
    /*    mesher.generateConstrained(samplesPerEdge_,
                                   samplesPerSurface_);*/

    // Build connectivity
    context.rebuildConnectivity();

    SPDLOG_INFO("ConstrainedMesher: Mesh generation complete");
    SPDLOG_INFO("  Nodes: {}", context.getMeshData().getNodeCount());
    SPDLOG_INFO("  Elements: {}", context.getMeshData().getElementCount());
}

} // namespace Meshing
