#pragma once
#include <algorithm>
#include <array>
#include <functional>

namespace Meshing
{

/**
 * @brief Canonical representation of an undirected edge
 *
 * An edge is defined by 2 nodes, but can be represented in either order:
 * [1, 2] or [2, 1]. EdgeKey stores nodes in sorted order so the same edge
 * always has the same key, regardless of which element or face it came from
 * -- mirrors FaceKey's role for triangular faces.
 */
struct EdgeKey
{
    std::array<size_t, 2> nodeIds; // Always sorted

    EdgeKey(size_t n0, size_t n1)
    {
        nodeIds = {n0, n1};
        std::sort(nodeIds.begin(), nodeIds.end());
    }

    bool operator==(const EdgeKey& other) const
    {
        return nodeIds == other.nodeIds;
    }

    bool operator<(const EdgeKey& other) const
    {
        return nodeIds < other.nodeIds;
    }
};

struct EdgeKeyHash
{
    size_t operator()(const EdgeKey& key) const
    {
        const size_t h1 = std::hash<size_t>{}(key.nodeIds[0]);
        const size_t h2 = std::hash<size_t>{}(key.nodeIds[1]);
        return h1 ^ (h2 << 1);
    }
};

} // namespace Meshing
