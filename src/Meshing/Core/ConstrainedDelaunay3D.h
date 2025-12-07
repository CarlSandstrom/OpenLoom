#pragma once

#include <array>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Common/Types.h"
#include "ConstrainedDelaunay2D.h"
#include "ConstrainedDelaunayHelper.h"
#include "ConstraintStructures.h"
#include "Geometry/Base/GeometryCollection.h"
#include "Meshing/Core/Computer.h"
#include "Meshing/Core/MeshingContext.h"
#include "Meshing/Core/MeshingContext2D.h"
#include "Meshing/Data/MeshData.h"
#include "Meshing/Data/MeshOperations.h"
#include "Meshing/Data/TetrahedralElement.h"
#include "Topology/Topology.h"

namespace Meshing
{

/**
 * @brief Constrained 3D Delaunay triangulation
 *
 * Provides unconstrained and constrained 3D Delaunay triangulation.
 * Uses classical CDT approach: directly force segments into the mesh.
 *
 * Integration with project architecture:
 * - Uses MeshingContext for data access
 * - Uses MeshOperations (friend pattern) for modifications
 * - Uses Computer for geometric calculations
 * - Respects Topology/Geometry separation
 */
class ConstrainedDelaunay3D
{
public:
    explicit ConstrainedDelaunay3D(MeshingContext& context);
    ~ConstrainedDelaunay3D() = default;

    void initialize(const std::vector<Point3D>& points);
    void insertVertex(const Point3D& point);

    const MeshData& getMeshData() const { return meshData_; }
    MeshData& getMeshData() { return meshData_; }

    const std::unordered_set<size_t>& getActiveTetrahedronIds() const { return activeTetrahedra_; }
    bool isElementActive(size_t elementId) const;

    Computer& getComputer() { return computer_; }
    const Computer& getComputer() const { return computer_; }

    const TetrahedralElement* getTetrahedralElement(size_t elementId) const;

    bool isDelaunay() const;

    /**
     * @brief Generate constrained mesh from topology
     *
     * Process:
     * 1. Insert corner nodes from topology
     * 2. Sample and insert edge nodes
     * 3. Sample and triangulate surface nodes
     * 4. Create initial Delaunay triangulation
     * 5. Force each constraint segment into mesh
     * 6. Force each constraint subfacet into mesh
     */
    void generateConstrained(size_t samplesPerEdge = 10,
                             size_t samplesPerSurface = 5);

    /**
     * @brief Force a segment to appear in the mesh
     */
    void forceSegment(size_t startNodeId, size_t endNodeId);

    /**
     * @brief Force a triangular facet to appear in the mesh
     */
    void forceFacet(size_t n0, size_t n1, size_t n2);

protected:
    ConstraintSet constraints_;

    // Maps topology IDs to mesh node IDs
    std::unordered_map<std::string, size_t> topologyToNodeId_;

    // Track which constraints have been satisfied
    std::unordered_set<std::pair<size_t, size_t>, PairHash> satisfiedSegments_;
    std::unordered_set<std::array<size_t, 3>, TriangleHash> satisfiedFacets_;

    /**
     * @brief Extract constraints from topology/geometry
     */
    void extractConstraints_(const Topology::Topology& topology,
                             const Geometry::GeometryCollection& geometry,
                             size_t samplesPerEdge,
                             size_t samplesPerSurface);

    /**
     * @brief Insert nodes for all corners in topology
     */
    void insertCornerNodes_(const Topology::Topology& topology,
                            const Geometry::GeometryCollection& geometry);

    /**
     * @brief Sample and insert nodes along edges
     */
    void insertEdgeNodes_(const Topology::Topology& topology,
                          const Geometry::GeometryCollection& geometry,
                          size_t samplesPerEdge);

    /**
     * @brief Sample and triangulate surface nodes
     */
    void triangulateSurfaces_(const Topology::Topology& topology,
                              const Geometry::GeometryCollection& geometry,
                              size_t samplesPerSurface);

    /**
     * @brief Recover all constraint segments in the mesh
     */
    void recoverSegments_();

    /**
     * @brief Recover all constraint facets in the mesh
     */
    void recoverFacets_();

    /**
     * @brief Triangulate a surface in 2D parametric space using constrained Delaunay
     */
    std::vector<std::array<size_t, 3>> triangulateSurface2D_(
        const std::vector<size_t>& boundaryNodeIds,
        const std::vector<size_t>& interiorNodeIds,
        const Geometry::Surface* surface);

    /**
     * @brief Triangulate a surface using MeshingContext2D
     *
     * Alternative method that creates a MeshingContext2D from the surface
     * and uses ConstrainedDelaunay2D with the context-based workflow.
     *
     * @param surface The 3D surface geometry
     * @param topoSurface The 3D surface topology
     * @param samplesPerEdge Number of sample points per edge
     * @return Vector of triangles (node ID triplets)
     */
    std::vector<std::array<size_t, 3>> triangulateSurfaceWithContext_(
        const Geometry::Surface* surface,
        const Topology::Surface& topoSurface,
        size_t samplesPerEdge);

private:
    void createSuperTetrahedron(const std::vector<Point3D>& points);
    void removeSuperTetrahedron();
    std::vector<size_t> findConflictingTetrahedra(const Point3D& p) const;
    std::vector<std::array<size_t, 3>> findCavityBoundary(const std::vector<size_t>& conflicting) const;
    void retriangulate(size_t vertexNodeId, const std::vector<std::array<size_t, 3>>& boundary);

    Computer computer_;
    std::vector<size_t> superNodeIds_;
    std::unordered_set<size_t> activeTetrahedra_;

    MeshingContext& context_;
    MeshData& meshData_;
    MeshOperations& operations_;
};

} // namespace Meshing
