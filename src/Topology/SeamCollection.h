#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

namespace Topology3D
{

/**
 * @brief Tracks seam twin edges for periodic surfaces (cylinder, torus, etc.).
 *
 * A periodic CAD surface has a "seam" — a curve where the UV domain closes.
 * OpenCASCADE visits the seam edge twice when walking the face wire: once as
 * the original seam edge (at the seam boundary) and once as its twin (at the
 * opposite period boundary, traversed in reverse). The twin has no independent
 * 3D geometry; it exists only to close the 2D UV boundary rectangle.
 *
 * SeamCollection records which edges are seam twins, the seam direction (U or V),
 * and the precomputed UV positions of the twin edge's endpoints in the shifted
 * domain. The UV positions are computed from the 2D parametric curve of the
 * original seam edge and are needed because closed-loop seam edges (start == end
 * in 3D, e.g. torus) cannot have their UV endpoints recovered from projectPoint.
 */
class SeamCollection
{
public:
    enum class SeamDirection
    {
        U, // Seam closes the U period; twin points are shifted by uPeriod in U
        V  // Seam closes the V period; twin points are shifted by vPeriod in V
    };

    SeamCollection() = default;

    // Register a seam twin pair with its direction and UV endpoints in the shifted domain.
    // twinStartUV: UV of the twin edge's topology-start corner (= original end + shift).
    // twinEndUV:   UV of the twin edge's topology-end corner   (= original start + shift).
    void addPair(const std::string& originalEdgeId,
                 const std::string& twinEdgeId,
                 SeamDirection direction,
                 std::array<double, 2> twinStartUV,
                 std::array<double, 2> twinEndUV);

    bool isSeamTwin(const std::string& edgeId) const;

    // Returns the original edge ID for a seam twin.
    // Caller must verify isSeamTwin() first.
    const std::string& getOriginalEdgeId(const std::string& twinEdgeId) const;

    SeamDirection getSeamDirection(const std::string& twinEdgeId) const;

    // UV of the twin edge's start corner in the shifted parametric domain.
    std::array<double, 2> getTwinStartUV(const std::string& twinEdgeId) const;

    // UV of the twin edge's end corner in the shifted parametric domain.
    std::array<double, 2> getTwinEndUV(const std::string& twinEdgeId) const;

    // Returns all seam twin edge IDs.
    std::vector<std::string> getSeamTwinEdgeIds() const;

    bool empty() const;

private:
    std::unordered_map<std::string, std::string> twinToOriginal_;
    std::unordered_map<std::string, SeamDirection> twinToDirection_;
    std::unordered_map<std::string, std::array<double, 2>> twinToStartUV_;
    std::unordered_map<std::string, std::array<double, 2>> twinToEndUV_;
};

} // namespace Topology3D
