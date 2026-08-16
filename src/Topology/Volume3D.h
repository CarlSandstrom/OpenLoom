#pragma once

#include <string>
#include <vector>

namespace Topology3D
{

class Volume3D
{
public:
    Volume3D(const std::string& id,
             const std::vector<std::string>& boundarySurfaceIds,
             const std::vector<std::string>& adjacentVolumeIds = {});

    std::string getId() const;
    const std::vector<std::string>& getBoundarySurfaceIds() const;
    const std::vector<std::string>& getAdjacentVolumeIds() const;

private:
    std::string id_;
    std::vector<std::string> boundarySurfaceIds_;
    std::vector<std::string> adjacentVolumeIds_; // Neighboring volumes/phases, sharing a boundary surface
};

} // namespace Topology3D
