#include "TwinManager.h"

const std::string TwinManager::NO_SURFACE = "";

void TwinManager::registerTwin(const std::string& surfaceId, size_t n1, size_t n2,
                                const std::string& twinSurfaceId, size_t m1, size_t m2)
{
    twinMap_[{surfaceId,     n1, n2}] = {twinSurfaceId, m1, m2};
    twinMap_[{surfaceId,     n2, n1}] = {twinSurfaceId, m2, m1};
    twinMap_[{twinSurfaceId, m1, m2}] = {surfaceId,     n1, n2};
    twinMap_[{twinSurfaceId, m2, m1}] = {surfaceId,     n2, n1};
}

bool TwinManager::hasTwin(const std::string& surfaceId, size_t n1, size_t n2) const
{
    return twinMap_.contains({surfaceId, n1, n2});
}

std::optional<std::tuple<std::string, size_t, size_t>>
TwinManager::getTwin(const std::string& surfaceId, size_t n1, size_t n2) const
{
    auto it = twinMap_.find({surfaceId, n1, n2});
    if (it == twinMap_.end())
        return std::nullopt;
    return it->second;
}

void TwinManager::recordSplit(const std::string& surfaceId, size_t n1, size_t n2, size_t mid,
                               const std::string& twinSurfaceId, size_t m1, size_t m2, size_t twinMid)
{
    twinMap_.erase({surfaceId,     n1, n2});
    twinMap_.erase({surfaceId,     n2, n1});
    twinMap_.erase({twinSurfaceId, m1, m2});
    twinMap_.erase({twinSurfaceId, m2, m1});

    registerTwin(surfaceId, n1,  mid,       twinSurfaceId, m1,      twinMid);
    registerTwin(surfaceId, mid, n2,        twinSurfaceId, twinMid, m2);
}
