#pragma once

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
 * the original seam edge (at U=0) and once as its twin (at U=uPeriod, traversed
 * in reverse). The twin has no independent 3D geometry; it exists only to close
 * the 2D UV boundary rectangle.
 *
 * SeamCollection records which edges are seam twins and maps each twin back to
 * its original, keeping this knowledge out of the general Edge3D type.
 */
class SeamCollection
{
public:
    SeamCollection() = default;

    // Register that twinEdgeId is the seam twin of originalEdgeId.
    void addPair(const std::string& originalEdgeId, const std::string& twinEdgeId);

    bool isSeamTwin(const std::string& edgeId) const;

    // Returns the original edge ID for a seam twin.
    // Caller must verify isSeamTwin() first.
    const std::string& getOriginalEdgeId(const std::string& twinEdgeId) const;

    // Returns all seam twin edge IDs.
    std::vector<std::string> getSeamTwinEdgeIds() const;

    bool empty() const;

private:
    // twin edge ID → original edge ID
    std::unordered_map<std::string, std::string> twinToOriginal_;
};

} // namespace Topology3D
