#include "Corner.h"

namespace Topology {

Corner::Corner(const std::string& id,
               const std::vector<std::string>& connectedEdgeIds,
               const std::vector<std::string>& connectedSurfaceIds)
    : id_(id),
      connectedEdgeIds_(connectedEdgeIds),
      connectedSurfaceIds_(connectedSurfaceIds) {
}

std::string Corner::getId() const {
    return id_;
}

const std::vector<std::string>& Corner::getConnectedEdgeIds() const {
    return connectedEdgeIds_;
}

const std::vector<std::string>& Corner::getConnectedSurfaceIds() const {
    return connectedSurfaceIds_;
}

} // namespace Topology