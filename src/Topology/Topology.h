#pragma once

#include "Corner.h"
#include "Edge.h"
#include "Surface.h"
#include <string>
#include <unordered_map>
#include <vector>

namespace Topology
{

class Topology
{
public:
    Topology(const std::unordered_map<std::string, Surface>& surfaces,
             const std::unordered_map<std::string, Edge>& edges,
             const std::unordered_map<std::string, Corner>& corners);

    // Entity access
    const Surface& getSurface(const std::string& id) const;
    const Edge& getEdge(const std::string& id) const;
    const Corner& getCorner(const std::string& id) const;

    // Global queries
    std::vector<std::string> getAllSurfaceIds() const;
    std::vector<std::string> getAllEdgeIds() const;
    std::vector<std::string> getAllCornerIds() const;
    std::vector<std::string> getBoundaryEdgeIds() const;
    std::vector<std::string> getNonManifoldEdgeIds() const;

    // Validation
    bool isValid() const;
    bool isManifold() const;

private:
    std::unordered_map<std::string, Surface> surfaces_;
    std::unordered_map<std::string, Edge> edges_;
    std::unordered_map<std::string, Corner> corners_;
};

} // namespace Topology