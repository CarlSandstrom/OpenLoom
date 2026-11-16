#pragma once

#include <string>
#include <vector>

namespace Meshing
{

class MeshData2D;

/**
 * @brief Verifies the validity of a 2D mesh
 *
 * Checks that:
 * - All triangular elements have counter-clockwise orientation
 * - No elements overlap
 */
class MeshVerifier2D
{
public:
    struct VerificationResult
    {
        bool isValid;
        std::vector<std::string> errors;
        std::vector<std::string> warnings;
    };

    /**
     * @brief Verify the complete mesh
     * @param meshData Reference to the 2D mesh data to verify
     * @return Verification result with validity status and error/warning messages
     */
    static VerificationResult verify(const MeshData2D& meshData);
};

} // namespace Meshing
