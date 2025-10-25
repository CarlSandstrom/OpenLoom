#include "Surface.h"

namespace Topology {

Surface::Surface(const std::string& id,
                 const std::vector<std::string>& boundaryEdgeIds,
                 const std::vector<std::string>& cornerIds,
                 const std::vector<std::string>& adjacentSurfaceIds,
                 const std::vector<std::vector<std::string>>& edgeLoops)
    : id_(id),
      boundaryEdgeIds_(boundaryEdgeIds),
      cornerIds_(cornerIds),
      adjacentSurfaceIds_(adjacentSurfaceIds),
      edgeLoops_(edgeLoops) {
}

std::string Surface::getId() const {
    return id_;
}

const std::vector<std::string>& Surface::getBoundaryEdgeIds() const {
    return boundaryEdgeIds_;
}

const std::vector<std::string>& Surface::getCornerIds() const {
    return cornerIds_;
}

const std::vector<std::string>& Surface::getAdjacentSurfaceIds() const {
    return adjacentSurfaceIds_;
}

const std::vector<std::vector<std::string>>& Surface::getEdgeLoops() const {
    return edgeLoops_;
}

} // namespace Topology