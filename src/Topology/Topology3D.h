#pragma once

#include "Corner3D.h"
#include "Edge3D.h"
#include "SeamCollection.h"
#include "Surface3D.h"
#include "Volume3D.h"
#include <string>
#include <unordered_map>
#include <vector>

namespace Topology3D
{

class Topology3D
{
public:
    Topology3D(const std::unordered_map<std::string, Surface3D>& surfaces,
               const std::unordered_map<std::string, Edge3D>& edges,
               const std::unordered_map<std::string, Corner3D>& corners,
               SeamCollection seams = {},
               const std::unordered_map<std::string, Volume3D>& volumes = {});

    // Entity access
    const Surface3D& getSurface(const std::string& id) const;
    const Edge3D& getEdge(const std::string& id) const;

    // Global queries
    std::vector<std::string> getAllSurfaceIds() const;
    std::vector<std::string> getAllEdgeIds() const;
    std::vector<std::string> getAllCornerIds() const;

    const SeamCollection& getSeamCollection() const;

private:
    std::unordered_map<std::string, Surface3D> surfaces_;
    std::unordered_map<std::string, Edge3D> edges_;
    std::unordered_map<std::string, Corner3D> corners_;
    SeamCollection seams_;
    std::unordered_map<std::string, Volume3D> volumes_;
};

} // namespace Topology3D
