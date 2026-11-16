#include "PeriodicOffsetTable2D.h"

#include "Common/Exceptions/GeometryException.h"

#include <cassert>
#include <string>

namespace Meshing
{

PeriodicOffsetTable2D::PeriodicOffsetTable2D(const PeriodicDomainConfiguration& configuration) :
    configuration_(configuration)
{
}

void PeriodicOffsetTable2D::setOffsets(ElementId triangleId,
                                     const std::array<PeriodicOffset, 3>& offsets)
{
    for (auto& offset : offsets)
    {
        assert(configuration_.uPeriodic || offset.u == 0);
        assert(configuration_.vPeriodic || offset.v == 0);
    }

    table_[triangleId] = offsets;
}

void PeriodicOffsetTable2D::removeOffsets(ElementId triangleId)
{
    table_.erase(triangleId);
}

const std::array<PeriodicOffset, 3>& PeriodicOffsetTable2D::getOffsets(ElementId triangleId) const
{
    auto entry = table_.find(triangleId);
    if (entry == table_.end())
    {
        OPENLOOM_THROW_ENTITY_NOT_FOUND("PeriodicOffset", std::to_string(triangleId.value()));
    }
    return entry->second;
}

bool PeriodicOffsetTable2D::hasOffsets(ElementId triangleId) const
{
    return table_.contains(triangleId);
}

} // namespace Meshing
