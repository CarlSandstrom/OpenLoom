#pragma once

#include "Meshing/Data/3D/MeshData3D.h"
#include <string>
#include <vector>

class TwinManager;

namespace Meshing
{

class FacetTriangulationManager;

/**
 * @brief Verification result containing details about any mesh issues found
 */
struct MeshVerificationResult
{
    bool isValid = true;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    size_t nodeCount = 0;
    size_t elementCount = 0;
    size_t degenerateElementCount = 0;
    size_t invertedElementCount = 0;
    size_t orphanNodeCount = 0;
};

/**
 * @brief Verifies the integrity of a 3D tetrahedral mesh
 *
 * This class performs various validation checks on a tetrahedral mesh to ensure
 * it is valid and well-formed. It can be used during development to catch issues
 * early after mesh operations.
 *
 * Checks performed:
 * - All element node references are valid
 * - No degenerate tetrahedra (zero or negative volume)
 * - No inverted tetrahedra (incorrect orientation)
 * - No orphan nodes (nodes not referenced by any element)
 * - No duplicate elements
 * - No NaN or Inf coordinates
 */
class MeshVerifier3D
{
public:
    /**
     * @brief Construct a mesh verifier for the given mesh data
     * @param meshData The mesh to verify
     */
    explicit MeshVerifier3D(const MeshData3D& meshData);

    /**
     * @brief Perform all verification checks
     * @return MeshVerificationResult containing details of any issues found
     */
    MeshVerificationResult verify() const;

    /**
     * @brief Verify that all twin segment pairs are consistent across adjacent faces.
     *
     * For each directed segment pair registered in the TwinManager, checks:
     * 1. Both endpoint node IDs exist in their respective face meshes.
     * 2. The twin map is symmetric: every A→B entry has a matching B→A entry.
     *    Symmetry implies equal subdivision counts on all shared edges, because
     *    recordSplit() always splits both twins together, so if the map is
     *    symmetric every sub-segment on one side has exactly one twin on the other.
     *
     * @param twinManager   The TwinManager populated during surface discretization.
     * @param facetManager  The FacetTriangulationManager owning all per-face meshes.
     * @return MeshVerificationResult with any inconsistencies reported as errors.
     */
    static MeshVerificationResult verifyTwinConsistency(const TwinManager& twinManager,
                                                        const FacetTriangulationManager& facetManager);

    /**
     * @brief Minimum volume threshold for degenerate detection
     */
    static constexpr double MIN_VALID_VOLUME = 1e-20;

private:
    const MeshData3D& meshData_;
    mutable MeshVerificationResult lastResult_;

    bool verifyNodeReferences() const;
    bool verifyNoDegenerateElements() const;
    bool verifyNoInvertedElements() const;
    bool verifyNoOrphanNodes() const;
    bool verifyValidCoordinates() const;
    double computeSignedVolume(size_t nodeId1, size_t nodeId2,
                               size_t nodeId3, size_t nodeId4) const;
};

} // namespace Meshing
