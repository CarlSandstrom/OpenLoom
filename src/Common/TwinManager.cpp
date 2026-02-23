#include "TwinManager.h"

void TwinManager::registerTwin(size_t n1, size_t n2, size_t m1, size_t m2)
{
    twinMap_[{n1, n2}] = {m1, m2};
    twinMap_[{n2, n1}] = {m2, m1};
    twinMap_[{m1, m2}] = {n1, n2};
    twinMap_[{m2, m1}] = {n2, n1};
}

bool TwinManager::hasTwin(size_t n1, size_t n2) const
{
    return twinMap_.contains({n1, n2});
}

std::optional<std::pair<size_t, size_t>> TwinManager::getTwin(size_t n1, size_t n2) const
{
    auto it = twinMap_.find({n1, n2});
    if (it == twinMap_.end())
    {
        return std::nullopt;
    }
    return it->second;
}

void TwinManager::recordSplit(size_t n1, size_t n2, size_t mid,
                               size_t m1, size_t m2, size_t twinMid)
{
    // Remove the four directed entries for the old pair
    twinMap_.erase({n1, n2});
    twinMap_.erase({n2, n1});
    twinMap_.erase({m1, m2});
    twinMap_.erase({m2, m1});

    // Register the two new sub-segment pairs
    registerTwin(n1, mid, m1, twinMid);
    registerTwin(mid, n2, twinMid, m2);
}
