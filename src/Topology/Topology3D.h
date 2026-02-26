#pragma once

#include "Corner3D.h"
#include "Edge3D.h"
#include "SeamCollection.h"
#include "Surface3D.h"
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
               SeamCollection seams = {});

    // Entity access
    const Surface3D& getSurface(const std::string& id) const;
    const Edge3D& getEdge(const std::string& id) const;
    const Corner3D& getCorner(const std::string& id) const;

    // Global queries
    std::vector<std::string> getAllSurfaceIds() const;
    std::vector<std::string> getAllEdgeIds() const;
    std::vector<std::string> getAllCornerIds() const;
    std::vector<std::string> getBoundaryEdgeIds() const;
    std::vector<std::string> getNonManifoldEdgeIds() const;

    const SeamCollection& getSeamCollection() const;

    // Validation
    bool isValid() const;
    bool isManifold() const;

private:
    std::unordered_map<std::string, Surface3D> surfaces_;
    std::unordered_map<std::string, Edge3D> edges_;
    std::unordered_map<std::string, Corner3D> corners_;
    SeamCollection seams_;
};

} // namespace Topology3D
