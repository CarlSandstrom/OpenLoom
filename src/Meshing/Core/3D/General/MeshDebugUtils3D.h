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
 * @brief Conditionally export a 3D mesh to VTU when EXPORT_MESH_EACH_ITERATION is enabled.
 *
 * Writes "{filenamePrefix}_{counter}.vtu" and increments the counter.
 *
 * @param meshData The mesh data to export
 * @param filenamePrefix Prefix for the exported VTU filename
 * @param exportCounter Counter used as the filename index (not modified).
 */
void exportMesh3D(MeshData3D& meshData,
                  const std::string& filenamePrefix,
                  size_t exportCounter);

/**
 * @brief Conditionally verify a 3D mesh when CHECK_MESH_EACH_ITERATION is enabled.
 *
 * Runs phase-appropriate invariant checks and throws on failure.
 * Verification checks are cumulative — later phases include all earlier checks.
 *
 * @param meshData The mesh data to verify
 * @param phase Current meshing phase (determines which invariants to check)
 * @param filenamePrefix Used in log messages to identify the caller
 * @param stepNumber Current iteration number for log output
 * @param qualityBound Quality ratio bound B for Refined/PostProcessed phases (ignored for earlier phases)
 */
void verifyMesh3D(MeshData3D& meshData,
                  MeshingPhase3D phase,
                  const std::string& filenamePrefix,
                  size_t stepNumber,
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
