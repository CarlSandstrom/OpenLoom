#include "Edge.h"

namespace Topology {

Edge::Edge(const std::string& id,
           const std::string& startCornerId,
           const std::string& endCornerId,
           const std::vector<std::string>& adjacentSurfaceIds)
    : id_(id),
      startCornerId_(startCornerId),
      endCornerId_(endCornerId),
      adjacentSurfaceIds_(adjacentSurfaceIds) {
}

std::string Edge::getId() const {
    return id_;
}

std::string Edge::getStartCornerId() const {
    return startCornerId_;
}

std::string Edge::getEndCornerId() const {
    return endCornerId_;
}

const std::vector<std::string>& Edge::getAdjacentSurfaceIds() const {
    return adjacentSurfaceIds_;
}

bool Edge::isBoundaryEdge() const {
    return adjacentSurfaceIds_.size() == 1;
}

bool Edge::isManifold() const {
    return adjacentSurfaceIds_.size() <= 2;
}

} // namespace Topology