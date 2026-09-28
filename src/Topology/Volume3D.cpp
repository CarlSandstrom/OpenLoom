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

} // namespace Topology3D
