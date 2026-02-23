#pragma once

#include <map>
#include <optional>
#include <utility>

/**
 * @brief Tracks paired ("twin") mesh segments that must be split synchronously.
 *
 * Used for periodic boundary conditions: when a segment on one boundary edge
 * is split during refinement, the registered twin segment on the opposite
 * boundary edge must be split at the corresponding parametric position.
 *
 * Segments are stored as directed pairs (from, to) so that endpoint
 * correspondence is preserved across splits. All four lookup directions for
 * any registered pair (A→B)↔(C→D) are stored automatically.
 */
class TwinManager
{
public:
    /**
     * @brief Register two segments as twins.
     *
     * Records that n1↔m1 and n2↔m2, meaning: when segment {n1,n2} is split
     * at a new node `mid`, the twin segment {m1,m2} must be split at `twinMid`
     * such that mid↔twinMid.
     *
     * Inserts all four directed entries so lookups work in any direction:
     *   (n1,n2)→(m1,m2), (n2,n1)→(m2,m1),
     *   (m1,m2)→(n1,n2), (m2,m1)→(n2,n1).
     */
    void registerTwin(size_t n1, size_t n2, size_t m1, size_t m2);

    /**
     * @brief Return true if segment {n1,n2} (in either direction) has a twin.
     */
    bool hasTwin(size_t n1, size_t n2) const;

    /**
     * @brief Return the directed twin of segment n1→n2.
     *
     * If segment n1→n2 is registered, returns (m1, m2) such that n1↔m1 and
     * n2↔m2. Returns nullopt if the segment is not registered.
     */
    std::optional<std::pair<size_t, size_t>> getTwin(size_t n1, size_t n2) const;

    /**
     * @brief Update registry after a split.
     *
     * Call this after segment {n1,n2} was split at node `mid` and its twin
     * {m1,m2} was split at node `twinMid`. Removes the old registration and
     * registers the two new sub-segment pairs:
     *   {n1, mid} ↔ {m1, twinMid}
     *   {mid, n2} ↔ {twinMid, m2}
     */
    void recordSplit(size_t n1, size_t n2, size_t mid,
                     size_t m1, size_t m2, size_t twinMid);

private:
    // Key: directed (from, to). Value: directed twin (twinFrom, twinTo).
    std::map<std::pair<size_t, size_t>, std::pair<size_t, size_t>> twinMap_;
};
