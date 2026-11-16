#pragma once

#include "Common/Types.h"

#include <map>
#include <optional>
#include <string>
#include <tuple>

/**
 * @brief Tracks paired ("twin") mesh segments that must be split synchronously.
 *
 * Segments are identified by (surfaceId, node0, node1) triples. Use NO_SURFACE as
 * the surface ID for pure-2D contexts (for example periodic boundaries on a single face).
 *
 * When a segment on one boundary is split during refinement, the registered
 * twin segment must be split at the corresponding parametric position.
 *
 * Segments are stored as directed pairs (from, to) so that endpoint
 * correspondence is preserved across splits. All four lookup directions for
 * any registered pair (A→B)↔(C→D) are stored automatically.
 */
class TwinManager2D
{
public:
    /// A directed segment (surfaceId, node0, node1).
    using SegmentKey = std::tuple<std::string, Meshing::NodeId, Meshing::NodeId>;

    /// Surface ID to use for pure-2D (no-surface) contexts.
    static const std::string NO_SURFACE;

    /**
     * @brief Register two segments as twins.
     *
     * Records that node0↔twinNode0 and node1↔twinNode1, meaning: when segment
     * {node0,node1} on surfaceId is split at a new node `midNode`, the twin segment
     * {twinNode0,twinNode1} on twinSurfaceId must be split at `twinMidNode` such
     * that midNode↔twinMidNode.
     *
     * Inserts all four directed entries so lookups work in any direction.
     */
    void registerTwin(const std::string& surfaceId, Meshing::NodeId node0, Meshing::NodeId node1,
                      const std::string& twinSurfaceId, Meshing::NodeId twinNode0, Meshing::NodeId twinNode1);

    /**
     * @brief Return true if segment {node0,node1} on surfaceId (in either direction) has a twin.
     */
    bool hasTwin(const std::string& surfaceId, Meshing::NodeId node0, Meshing::NodeId node1) const;

    /**
     * @brief Return the directed twin of segment node0→node1 on surfaceId.
     *
     * Returns (twinSurfaceId, twinNode0, twinNode1) such that node0↔twinNode0 and
     * node1↔twinNode1. Returns nullopt if the segment is not registered.
     */
    std::optional<SegmentKey> getTwin(const std::string& surfaceId, Meshing::NodeId node0, Meshing::NodeId node1) const;

    /**
     * @brief Update registry after a split.
     *
     * Call this after segment {node0,node1} on surfaceId was split at `midNode`
     * and its twin {twinNode0,twinNode1} on twinSurfaceId was split at `twinMidNode`.
     * Removes the old registration and registers the two new sub-segment pairs.
     */
    void recordSplit(const std::string& surfaceId, Meshing::NodeId node0, Meshing::NodeId node1, Meshing::NodeId midNode,
                     const std::string& twinSurfaceId, Meshing::NodeId twinNode0, Meshing::NodeId twinNode1,
                     Meshing::NodeId twinMidNode);

    /**
     * @brief Return all registered directed segment pairs.
     *
     * Each entry maps a directed segment key (surfaceId, node0, node1) to its twin
     * (twinSurfaceId, twinNode0, twinNode1). All four directions of each logical twin
     * pair are present (see registerTwin). Used for verification.
     */
    const std::map<SegmentKey, SegmentKey>& getAllPairs() const;

private:
    std::map<SegmentKey, SegmentKey> twinMap_;
};
