#pragma once

#include <cstddef>
#include <string>

namespace Meshing
{

class MeshData3D;
class DiscretizationResult3D;

/**
 * @brief Phases of the 3D meshing algorithm, each with cumulative validation invariants.
 *
 * Each phase accumulates the checks from all previous phases:
 * - InitialDelaunay: basic mesh integrity (no degenerate/inverted tets, valid refs)
 * - SegmentRecovery: + every subsegment exists as a mesh edge
 * - FacetRecovery:   + every subfacet exists as a mesh face
 * - ExteriorRemoved: + basic integrity after exterior tet removal
 * - Refined:         + quality bound satisfied (no skinny tets)
 * - PostProcessed:   + no slivers (dihedral angle threshold)
 */
enum class MeshingPhase3D
{
    InitialDelaunay,
    SegmentRecovery,
    FacetRecovery,
    ExteriorRemoved,
    Refined,
    PostProcessed
};

/**
 * @brief Conditionally export and verify a 3D mesh based on debug flags and meshing phase.
 *
 * When EXPORT_MESH_EACH_ITERATION is enabled, exports the mesh to a VTU file
 * named "{filenamePrefix}_{counter}.vtu" and increments the counter.
 * When CHECK_MESH_EACH_ITERATION is enabled, runs phase-appropriate verification
 * checks and throws on failure.
 *
 * Verification checks are cumulative — later phases include all earlier checks.
 *
 * @param meshData The mesh data to export/verify
 * @param phase Current meshing phase (determines which invariants to check)
 * @param filenamePrefix Prefix for the exported VTU filename
 * @param exportCounter Counter tracking export iterations (incremented on export)
 * @param qualityBound Quality ratio bound B for Refined/PostProcessed phases (ignored for earlier phases)
 */
void exportAndVerifyMesh3D(MeshData3D& meshData,
                           MeshingPhase3D phase,
                           const std::string& filenamePrefix,
                           size_t& exportCounter,
                           double qualityBound = 0.0);

/**
 * @brief Conditionally export discretized boundary edges based on EXPORT_MESH_EACH_ITERATION.
 *
 * Exports the edge mesh to "{filename}" when the flag is enabled.
 *
 * @param result    The discretization result containing edge points and indices
 * @param filename  Output VTU filename
 */
void exportEdgeMesh3D(const DiscretizationResult3D& result, const std::string& filename);

} // namespace Meshing
