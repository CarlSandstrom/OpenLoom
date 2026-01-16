#pragma once

#include <memory>
#include <vector>

#include "GeometryStructures3D.h"

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
 * data structures. Also manages constraint lists (subsegments and subfacets)
 * needed for Shewchuk's 3D Delaunay refinement algorithm.
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

    // Clear mesh (data + connectivity)
    void clearMesh();

    // ========== Constraint Management for Shewchuk Refinement ==========

    /**
     * @brief Get the constrained subsegments (boundary edges)
     */
    const std::vector<ConstrainedSubsegment3D>& getConstrainedSubsegments() const;
    std::vector<ConstrainedSubsegment3D>& getConstrainedSubsegments();

    /**
     * @brief Get the constrained subfacets (boundary faces)
     */
    const std::vector<ConstrainedSubfacet3D>& getConstrainedSubfacets() const;
    std::vector<ConstrainedSubfacet3D>& getConstrainedSubfacets();

    /**
     * @brief Add a constrained subsegment
     */
    void addConstrainedSubsegment(const ConstrainedSubsegment3D& subsegment);

    /**
     * @brief Add a constrained subfacet
     */
    void addConstrainedSubfacet(const ConstrainedSubfacet3D& subfacet);

    /**
     * @brief Set all constrained subsegments at once
     */
    void setConstrainedSubsegments(std::vector<ConstrainedSubsegment3D> subsegments);

    /**
     * @brief Set all constrained subfacets at once
     */
    void setConstrainedSubfacets(std::vector<ConstrainedSubfacet3D> subfacets);

private:
    const Geometry3D::GeometryCollection3D* geometry_ = nullptr;
    const Topology3D::Topology3D* topology_ = nullptr;

    std::unique_ptr<MeshData3D> meshData_;
    std::unique_ptr<MeshConnectivity> connectivity_;
    std::unique_ptr<MeshMutator3D> meshMutator_;
    std::unique_ptr<MeshOperations3D> meshOperations_;

    // Constraint lists for Shewchuk refinement
    std::vector<ConstrainedSubsegment3D> constrainedSubsegments_;
    std::vector<ConstrainedSubfacet3D> constrainedSubfacets_;

    void ensureInitialized();
};

} // namespace Meshing
