#pragma once
#include <algorithm>
#include <array>
#include <functional>

namespace Meshing
{

/**
 * @brief Canonical representation of a triangular face
 *
 * A face is defined by 3 nodes, but can be represented in different orders:
 * - [1, 2, 3]
 * - [2, 3, 1]
 * - [3, 1, 2]
 *
 * FaceKey stores nodes in sorted order so the same face always has
 * the same key, regardless of which element it came from
 */
struct FaceKey
{
    std::array<size_t, 3> nodeIds; // Always sorted

    // Constructor that automatically sorts
    FaceKey(size_t n0, size_t n1, size_t n2)
    {
        nodeIds = {n0, n1, n2};
        std::sort(nodeIds.begin(), nodeIds.end());
    }

    FaceKey(const std::array<size_t, 3>& nodes) :
        nodeIds(nodes)
    {
        std::sort(nodeIds.begin(), nodeIds.end());
    }

    // Comparison for use in map
    bool operator==(const FaceKey& other) const
    {
        return nodeIds == other.nodeIds;
    }

    bool operator<(const FaceKey& other) const
    {
        return nodeIds < other.nodeIds;
    }
};

/**
 * @brief Hash function for FaceKey to use in unordered_map
 */
struct FaceKeyHash
{
    size_t operator()(const FaceKey& key) const
    {
        // Combine hashes of the three node IDs
        size_t h1 = std::hash<size_t>{}(key.nodeIds[0]);
        size_t h2 = std::hash<size_t>{}(key.nodeIds[1]);
        size_t h3 = std::hash<size_t>{}(key.nodeIds[2]);

        // Simple hash combining (boost-style)
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

/**
 * @brief Helper function to create FaceKey from node IDs
 */
inline FaceKey makeFaceKey(size_t n0, size_t n1, size_t n2)
{
    return FaceKey(n0, n1, n2);
}

inline FaceKey makeFaceKey(const std::array<size_t, 3>& nodes)
{
    return FaceKey(nodes);
}

} // namespace Meshing