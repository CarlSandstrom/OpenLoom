#pragma once

#include <memory>

namespace Geometry3D
{
class GeometryCollection3D;
}
namespace Topology3D
{
class Topology3D;
}

namespace Meshing
{
class MeshData3D;
class MeshConnectivity;
class MeshMutator3D;

// Central orchestrator holding references to geometry and topology and
// owning mesh-specific mutable data structures.
class MeshingContext3D
{
public:
    MeshingContext3D(const Geometry3D::GeometryCollection3D& geometry,
                     const Topology3D::Topology3D& topology);
    ~MeshingContext3D();

    // Non-owning references to input domains
    const Geometry3D::GeometryCollection3D& getGeometry() const { return geometry_; }
    const Topology3D::Topology3D& getTopology() const { return topology_; }

    // Access to mesh data structures
    MeshData3D& getMeshData();
    MeshConnectivity& getConnectivity();
    MeshMutator3D& getMutator();

    // Utility: rebuild connectivity after large changes
    void rebuildConnectivity();

    // Clear mesh (data + connectivity)
    void clearMesh();

private:
    const Geometry3D::GeometryCollection3D& geometry_;
    const Topology3D::Topology3D& topology_;

    std::unique_ptr<MeshData3D> meshData_;
    std::unique_ptr<MeshConnectivity> connectivity_;
    std::unique_ptr<MeshMutator3D> meshMutator_;

    void ensureInitialized();
};

} // namespace Meshing
