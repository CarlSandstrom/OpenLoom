#include "Meshing/Core/2D/Periodic/TwinManager2D.h"

const std::string TwinManager2D::NO_SURFACE = "";

void TwinManager2D::registerTwin(const std::string& surfaceId, size_t node0, size_t node1,
                               const std::string& twinSurfaceId, size_t twinNode0, size_t twinNode1)
{
    twinMap_[{surfaceId, node0, node1}] = {twinSurfaceId, twinNode0, twinNode1};
    twinMap_[{surfaceId, node1, node0}] = {twinSurfaceId, twinNode1, twinNode0};
    twinMap_[{twinSurfaceId, twinNode0, twinNode1}] = {surfaceId, node0, node1};
    twinMap_[{twinSurfaceId, twinNode1, twinNode0}] = {surfaceId, node1, node0};
}

bool TwinManager2D::hasTwin(const std::string& surfaceId, size_t node0, size_t node1) const
{
    return twinMap_.contains({surfaceId, node0, node1});
}

std::optional<TwinManager2D::SegmentKey> TwinManager2D::getTwin(const std::string& surfaceId, size_t node0, size_t node1) const
{
    auto twinEntry = twinMap_.find({surfaceId, node0, node1});
    if (twinEntry == twinMap_.end()) return std::nullopt;
    return twinEntry->second;
}

void TwinManager2D::recordSplit(const std::string& surfaceId, size_t node0, size_t node1, size_t midNode,
                              const std::string& twinSurfaceId, size_t twinNode0, size_t twinNode1,
                              size_t twinMidNode)
{
    twinMap_.erase({surfaceId, node0, node1});
    twinMap_.erase({surfaceId, node1, node0});
    twinMap_.erase({twinSurfaceId, twinNode0, twinNode1});
    twinMap_.erase({twinSurfaceId, twinNode1, twinNode0});

    registerTwin(surfaceId, node0, midNode, twinSurfaceId, twinNode0, twinMidNode);
    registerTwin(surfaceId, midNode, node1, twinSurfaceId, twinMidNode, twinNode1);
}

const std::map<TwinManager2D::SegmentKey, TwinManager2D::SegmentKey>& TwinManager2D::getAllPairs() const
{
    return twinMap_;
}
