#include "ConstrainedMesher.h"
#include "ConstrainedDelaunay3D.h"
#include "MeshingContext.h"
#include "Geometry/Base/GeometryCollection.h"
#include "Topology/Topology.h"
#include "spdlog/spdlog.h"

namespace Meshing
{

void ConstrainedMesher::generate(MeshingContext& context)
{
    SPDLOG_INFO("ConstrainedMesher: Starting mesh generation");
    SPDLOG_INFO("  Edge samples: {}", samplesPerEdge_);
    SPDLOG_INFO("  Surface samples: {}x{}", samplesPerSurface_, samplesPerSurface_);
    
    // Clear any existing mesh
    context.clearMesh();
    
    // Create constrained Delaunay mesher
    ConstrainedDelaunay3D mesher(context);
    
    // Generate constrained mesh respecting topology (edges AND faces)
    mesher.generateConstrained(context.getTopology(), 
                              context.getGeometry(),
                              samplesPerEdge_,
                              samplesPerSurface_);
    
    // Build connectivity
    context.rebuildConnectivity();
    
    SPDLOG_INFO("ConstrainedMesher: Mesh generation complete");
    SPDLOG_INFO("  Nodes: {}", context.getMeshData().getNodeCount());
    SPDLOG_INFO("  Elements: {}", context.getMeshData().getElementCount());
}

} // namespace Meshing
