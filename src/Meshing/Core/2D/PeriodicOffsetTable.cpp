#include "PeriodicOffsetTable.h"

#include "Common/Exceptions/GeometryException.h"

#include <cassert>
#include <string>

namespace Meshing
{

PeriodicOffsetTable::PeriodicOffsetTable(const PeriodicDomainConfig& config) :
    config_(config)
{
}

void PeriodicOffsetTable::setOffsets(size_t triangleId,
                                     const std::array<PeriodicOffset, 3>& offsets)
{
    for (const auto& offset : offsets)
    {
        if (!config_.uPeriodic)
            assert(offset.u == 0);
        if (!config_.vPeriodic)
            assert(offset.v == 0);
    }

    table_[triangleId] = offsets;
}

void PeriodicOffsetTable::removeOffsets(size_t triangleId)
{
    table_.erase(triangleId);
}

const std::array<PeriodicOffset, 3>& PeriodicOffsetTable::getOffsets(size_t triangleId) const
{
    auto it = table_.find(triangleId);
    if (it == table_.end())
    {
        OPENLOOM_THROW_ENTITY_NOT_FOUND("PeriodicOffset", std::to_string(triangleId));
    }
    return it->second;
}

bool PeriodicOffsetTable::hasOffsets(size_t triangleId) const
{
    return table_.count(triangleId) > 0;
}

} // namespace Meshing
