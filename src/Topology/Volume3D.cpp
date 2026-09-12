#include "Volume3D.h"

namespace Topology3D
{

Volume3D::Volume3D(const std::string& id,
                   const std::vector<std::string>& boundarySurfaceIds,
                   const std::vector<std::string>& adjacentVolumeIds) :
    id_(id),
    boundarySurfaceIds_(boundarySurfaceIds),
    adjacentVolumeIds_(adjacentVolumeIds)
{
}

std::string Volume3D::getId() const
{
    return id_;
}

const std::vector<std::string>& Volume3D::getBoundarySurfaceIds() const
{
    return boundarySurfaceIds_;
}

const std::vector<std::string>& Volume3D::getAdjacentVolumeIds() const
{
    return adjacentVolumeIds_;
}

} // namespace Topology3D
