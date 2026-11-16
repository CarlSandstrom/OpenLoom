#pragma once

#include "Common/Types.h"
#include "Meshing/Core/2D/Periodic/PeriodicDomain2D.h"

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
class PeriodicOffsetTable2D
{
public:
    explicit PeriodicOffsetTable2D(const PeriodicDomainConfiguration& configuration);

    /// Register the three per-vertex offsets for a newly created triangle.
    void setOffsets(ElementId triangleId, const std::array<PeriodicOffset, 3>& offsets);

    /// Remove the entry for a deleted triangle.
    void removeOffsets(ElementId triangleId);

    /// Returns the stored offsets for a triangle.
    /// Throws if no entry exists for this triangle.
    const std::array<PeriodicOffset, 3>& getOffsets(ElementId triangleId) const;

    /// Returns true if the table has an entry for this triangle.
    bool hasOffsets(ElementId triangleId) const;

private:
    PeriodicDomainConfiguration configuration_;
    std::unordered_map<ElementId, std::array<PeriodicOffset, 3>> table_;
};

} // namespace Meshing
