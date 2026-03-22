#pragma once

namespace Meshing
{

/// Integer offset applied to a node's canonical UV coordinate.
/// Each component is in {-1, 0, 1}: the number of periods to shift.
struct PeriodicOffset
{
    int u = 0;
    int v = 0;

    bool operator==(const PeriodicOffset& other) const
    {
        return u == other.u && v == other.v;
    }

    bool operator!=(const PeriodicOffset& other) const
    {
        return !(*this == other);
    }
};

/// Describes which UV directions of a 2D meshing domain are periodic,
/// and the period length in each direction.
struct PeriodicDomainConfig
{
    bool   uPeriodic = false;
    bool   vPeriodic = false;
    double uPeriod   = 0.0;
    double vPeriod   = 0.0;

    bool isPeriodic() const { return uPeriodic || vPeriodic; }
};

} // namespace Meshing
