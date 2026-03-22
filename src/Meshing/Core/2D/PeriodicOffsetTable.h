#pragma once

#include "PeriodicDomain2D.h"
#include <array>
#include <unordered_map>

namespace Meshing
{

/// Stores the three per-vertex periodic offsets for every triangle in a
/// periodic 2D triangulation.
///
/// In a periodic triangulation, some triangles straddle a period boundary.
/// Their vertices are stored at canonical UV coordinates in [0, period), but
/// the effective coordinates used for geometric predicates are shifted by an
/// integer number of periods.  This table maps each triangle ID to the three
/// offsets (one per vertex slot) that produce those effective coordinates.
///
/// Invariant: if the domain has uPeriodic == false, every stored offset must
/// have u == 0.  Likewise for v.  Violations are caught by assertion.
class PeriodicOffsetTable
{
public:
    explicit PeriodicOffsetTable(const PeriodicDomainConfig& config);

    /// Register the three per-vertex offsets for a newly created triangle.
    void setOffsets(size_t triangleId, const std::array<PeriodicOffset, 3>& offsets);

    /// Remove the entry for a deleted triangle.
    void removeOffsets(size_t triangleId);

    /// Returns the stored offsets for a triangle.
    /// Throws if no entry exists for this triangle.
    const std::array<PeriodicOffset, 3>& getOffsets(size_t triangleId) const;

    /// Returns true if the table has an entry for this triangle.
    bool hasOffsets(size_t triangleId) const;

private:
    PeriodicDomainConfig config_;
    std::unordered_map<size_t, std::array<PeriodicOffset, 3>> table_;
};

} // namespace Meshing
