#pragma once

#include <map>
#include <optional>
#include <string>
#include <tuple>

/**
 * @brief Tracks paired ("twin") mesh segments that must be split synchronously.
 *
 * Segments are identified by (surfaceId, n1, n2) triples. Use NO_SURFACE as
 * the surface ID for pure-2D contexts (e.g. periodic boundaries on a single face).
 *
 * When a segment on one boundary is split during refinement, the registered
 * twin segment must be split at the corresponding parametric position.
 *
 * Segments are stored as directed pairs (from, to) so that endpoint
 * correspondence is preserved across splits. All four lookup directions for
 * any registered pair (A→B)↔(C→D) are stored automatically.
 */
class TwinManager
{
public:
    /// Surface ID to use for pure-2D (no-surface) contexts.
    static const std::string NO_SURFACE;

    /**
     * @brief Register two segments as twins.
     *
     * Records that n1↔m1 and n2↔m2, meaning: when segment {n1,n2} on
     * surfaceId is split at a new node `mid`, the twin segment {m1,m2}
     * on twinSurfaceId must be split at `twinMid` such that mid↔twinMid.
     *
     * Inserts all four directed entries so lookups work in any direction.
     */
    void registerTwin(const std::string& surfaceId, size_t n1, size_t n2,
                      const std::string& twinSurfaceId, size_t m1, size_t m2);

    /**
     * @brief Return true if segment {n1,n2} on surfaceId (in either direction) has a twin.
     */
    bool hasTwin(const std::string& surfaceId, size_t n1, size_t n2) const;

    /**
     * @brief Return the directed twin of segment n1→n2 on surfaceId.
     *
     * Returns (twinSurfaceId, m1, m2) such that n1↔m1 and n2↔m2.
     * Returns nullopt if the segment is not registered.
     */
    std::optional<std::tuple<std::string, size_t, size_t>>
        getTwin(const std::string& surfaceId, size_t n1, size_t n2) const;

    /**
     * @brief Update registry after a split.
     *
     * Call this after segment {n1,n2} on surfaceId was split at node `mid`
     * and its twin {m1,m2} on twinSurfaceId was split at node `twinMid`.
     * Removes the old registration and registers the two new sub-segment pairs.
     */
    void recordSplit(const std::string& surfaceId, size_t n1, size_t n2, size_t mid,
                     const std::string& twinSurfaceId, size_t m1, size_t m2, size_t twinMid);

    /**
     * @brief Return all registered directed segment pairs.
     *
     * Each entry maps a directed segment key (surfaceId, n1, n2) to its twin
     * (twinSurfaceId, m1, m2). All four directions of each logical twin pair
     * are present (see registerTwin). Used for verification.
     */
    using SegmentKey = std::tuple<std::string, size_t, size_t>;
    const std::map<SegmentKey, SegmentKey>& getAllPairs() const { return twinMap_; }

private:
    std::map<SegmentKey, SegmentKey> twinMap_;
};
