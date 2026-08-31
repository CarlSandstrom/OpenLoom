#pragma once
#include <algorithm>
#include <array>
#include <cstddef>
#include <functional>

namespace Meshing
{

/**
 * @brief Canonical representation of a tetrahedron
 *
 * The 4-node counterpart of EdgeKey and FaceKey: node IDs are stored sorted,
 * so the same tetrahedron always produces the same key regardless of the
 * winding its element happened to be built with.
 *
 * Unlike an element ID, this identifies the tetrahedron by the only thing its
 * geometry actually depends on — which nodes it spans — which is what makes it
 * usable as the key of a cache that has to outlive individual elements.
 */
struct TetrahedronKey
{
    std::array<size_t, 4> nodeIds; // Always sorted

    TetrahedronKey(size_t n0, size_t n1, size_t n2, size_t n3)
    {
        nodeIds = {n0, n1, n2, n3};
        std::sort(nodeIds.begin(), nodeIds.end());
    }

    explicit TetrahedronKey(const std::array<size_t, 4>& nodes) :
        nodeIds(nodes)
    {
        std::sort(nodeIds.begin(), nodeIds.end());
    }

    bool operator==(const TetrahedronKey& other) const
    {
        return nodeIds == other.nodeIds;
    }

    bool operator<(const TetrahedronKey& other) const
    {
        return nodeIds < other.nodeIds;
    }
};

/**
 * @brief Hash function for TetrahedronKey to use in unordered_map
 */
struct TetrahedronKeyHash
{
    size_t operator()(const TetrahedronKey& key) const
    {
        // boost-style hash_combine rather than FaceKeyHash's small shifts:
        // std::hash<size_t> is the identity, and node IDs are dense small
        // integers, so shifting by 1..3 leaves the high bits of four such IDs
        // overlapping almost entirely.
        size_t seed = 0;
        for (const size_t nodeId : key.nodeIds)
            seed ^= std::hash<size_t>{}(nodeId) + 0x9e3779b97f4a7c15ULL + (seed << 6) + (seed >> 2);
        return seed;
    }
};

} // namespace Meshing
