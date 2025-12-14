#include "Topology3D.h"
#include "Common/Types.h"
#include <algorithm>
#include <stdexcept>

namespace Topology3D
{

Topology3D::Topology3D(const std::unordered_map<std::string, Surface3D>& surfaces,
                       const std::unordered_map<std::string, Edge3D>& edges,
                       const std::unordered_map<std::string, Corner3D>& corners) :
    surfaces_(surfaces),
    edges_(edges),
    corners_(corners)
{
}

const Surface3D& Topology3D::getSurface(const std::string& id) const
{
    auto it = surfaces_.find(id);
    if (it == surfaces_.end())
    {
        throw std::runtime_error("Surface with id '" + id + "' not found");
    }
    return it->second;
}

const Edge3D& Topology3D::getEdge(const std::string& id) const
{
    auto it = edges_.find(id);
    if (it == edges_.end())
    {
        throw std::runtime_error("Edge with id '" + id + "' not found");
    }
    return it->second;
}

const Corner3D& Topology3D::getCorner(const std::string& id) const
{
    auto it = corners_.find(id);
    if (it == corners_.end())
    {
        throw std::runtime_error("Corner with id '" + id + "' not found");
    }
    return it->second;
}

std::vector<std::string> Topology3D::getAllSurfaceIds() const
{
    std::vector<std::string> ids;
    ids.reserve(surfaces_.size());
    for (const auto& pair : surfaces_)
    {
        ids.push_back(pair.first);
    }
    return ids;
}

std::vector<std::string> Topology3D::getAllEdgeIds() const
{
    std::vector<std::string> ids;
    ids.reserve(edges_.size());
    for (const auto& pair : edges_)
    {
        ids.push_back(pair.first);
    }
    return ids;
}

std::vector<std::string> Topology3D::getAllCornerIds() const
{
    std::vector<std::string> ids;
    ids.reserve(corners_.size());
    for (const auto& pair : corners_)
    {
        ids.push_back(pair.first);
    }
    return ids;
}

std::vector<std::string> Topology3D::getBoundaryEdgeIds() const
{
    std::vector<std::string> boundaryIds;
    for (const auto& pair : edges_)
    {
        if (pair.second.isBoundaryEdge())
        {
            boundaryIds.push_back(pair.first);
        }
    }
    return boundaryIds;
}

std::vector<std::string> Topology3D::getNonManifoldEdgeIds() const
{
    std::vector<std::string> nonManifoldIds;
    for (const auto& pair : edges_)
    {
        if (!pair.second.isManifold())
        {
            nonManifoldIds.push_back(pair.first);
        }
    }
    return nonManifoldIds;
}

bool Topology3D::isValid() const
{
    // Check that all referenced entities exist
    for (const auto& surfacePair : surfaces_)
    {
        const Surface3D& surface = surfacePair.second;

        // Check that all boundary edges exist
        for (const std::string& edgeId : surface.getBoundaryEdgeIds())
        {
            if (edges_.find(edgeId) == edges_.end())
            {
                return false;
            }
        }

        // Check that all corners exist
        for (const std::string& cornerId : surface.getCornerIds())
        {
            if (corners_.find(cornerId) == corners_.end())
            {
                return false;
            }
        }

        // Check that all adjacent surfaces exist
        for (const std::string& adjacentId : surface.getAdjacentSurfaceIds())
        {
            if (surfaces_.find(adjacentId) == surfaces_.end())
            {
                return false;
            }
        }
    }

    // Check edges
    for (const auto& edgePair : edges_)
    {
        const Edge3D& edge = edgePair.second;

        // Check that start and end corners exist
        if (corners_.find(edge.getStartCornerId()) == corners_.end() ||
            corners_.find(edge.getEndCornerId()) == corners_.end())
        {
            return false;
        }

        // Check that all adjacent surfaces exist
        for (const std::string& surfaceId : edge.getAdjacentSurfaceIds())
        {
            if (surfaces_.find(surfaceId) == surfaces_.end())
            {
                return false;
            }
        }
    }

    // Check corners
    for (const auto& cornerPair : corners_)
    {
        const Corner3D& corner = cornerPair.second;

        // Check that all connected edges exist
        for (const std::string& edgeId : corner.getConnectedEdgeIds())
        {
            if (edges_.find(edgeId) == edges_.end())
            {
                return false;
            }
        }

        // Check that all connected surfaces exist
        for (const std::string& surfaceId : corner.getConnectedSurfaceIds())
        {
            if (surfaces_.find(surfaceId) == surfaces_.end())
            {
                return false;
            }
        }
    }

    return true;
}

bool Topology3D::isManifold() const
{
    // A topology is manifold if all edges are manifold (have at most 2 adjacent surfaces)
    for (const auto& pair : edges_)
    {
        if (!pair.second.isManifold())
        {
            return false;
        }
    }
    return true;
}

} // namespace Topology3D