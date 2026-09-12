#pragma once

#include "Common/Types.h"

#include <array>
#include <cstddef>
#include <unordered_map>
#include <vector>

namespace Meshing
{

/**
 * @brief Keeps a set of tetrahedra from being turned inside out by a sweep
 * that moves the nodes they share.
 *
 * A smoothing sweep proposes every move at once, so two moves that are each
 * harmless alone can still invert a tetrahedron they share. Each tetrahedron
 * is therefore judged with all of its nodes at their proposed positions, and
 * one that would invert has all of its nodes put back.
 *
 * Holds the node-to-tetrahedra index the check needs, so a caller running
 * several sweeps builds it once. The tetrahedra themselves are referenced,
 * not copied: the guard is meant to live no longer than the sweep it guards.
 */
class TetrahedronInversionGuard
{
public:
    /// tetrahedra index into the node vectors passed to revertInvertingMoves()
    /// and must outlive this guard. An empty set makes every call a no-op.
    explicit TetrahedronInversionGuard(const std::vector<std::array<std::size_t, 4>>& tetrahedra);

    /// Puts every node of every tetrahedron the proposed positions would
    /// invert back to its position in current, until no tetrahedron inverts.
    ///
    /// A node that is put back never moves again in this call, so this
    /// terminates, and a tetrahedron whose nodes are all back has exactly the
    /// orientation it started with.
    void revertInvertingMoves(const std::vector<Point3D>& current, std::vector<Point3D>& proposed) const;

private:
    const std::vector<std::array<std::size_t, 4>>& tetrahedra_;
    std::unordered_map<std::size_t, std::vector<std::size_t>> tetrahedraByNode_;
};

} // namespace Meshing
