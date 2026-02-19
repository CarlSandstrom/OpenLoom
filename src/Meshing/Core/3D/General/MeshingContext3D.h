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
class MeshOperations3D;

/**
 * @brief Central orchestrator for 3D tetrahedral meshing
 *
 * Holds references to geometry and topology and owns mesh-specific mutable
 * data structures. Constraints (subsegments, subfacets) are stored in
 * MeshData3D and mutated via MeshMutator3D.
 */
class MeshingContext3D
{
public:
    /**
     * @brief Construct context with geometry and topology
     */
    MeshingContext3D(const Geometry3D::GeometryCollection3D& geometry,
                     const Topology3D::Topology3D& topology);

    /**
     * @brief Construct a standalone context without geometry/topology
     *
     * Useful for testing or when working with meshes independently.
     */
    MeshingContext3D();

    ~MeshingContext3D();

    // Prevent copying
    MeshingContext3D(const MeshingContext3D&) = delete;
    MeshingContext3D& operator=(const MeshingContext3D&) = delete;

    // Allow moving
    MeshingContext3D(MeshingContext3D&&) noexcept;
    MeshingContext3D& operator=(MeshingContext3D&&) noexcept;

    // Non-owning references to input domains (may be null for standalone context)
    const Geometry3D::GeometryCollection3D* getGeometry() const { return geometry_; }
    const Topology3D::Topology3D* getTopology() const { return topology_; }

    // Access to mesh data structures
    MeshData3D& getMeshData();
    const MeshData3D& getMeshData() const;
    MeshConnectivity& getConnectivity();
    MeshMutator3D& getMutator();
    MeshOperations3D& getOperations();

    // Utility: rebuild connectivity after large changes
    void rebuildConnectivity();

    // Clear mesh (data + connectivity + constraints)
    void clearMesh();

private:
    const Geometry3D::GeometryCollection3D* geometry_ = nullptr;
    const Topology3D::Topology3D* topology_ = nullptr;

    std::unique_ptr<MeshData3D> meshData_;
    std::unique_ptr<MeshConnectivity> connectivity_;
    std::unique_ptr<MeshMutator3D> meshMutator_;
    std::unique_ptr<MeshOperations3D> meshOperations_;

    void ensureInitialized();
};

} // namespace Meshing
