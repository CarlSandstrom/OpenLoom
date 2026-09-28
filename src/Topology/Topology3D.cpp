#include "Topology3D.h"
#include "Common/Exceptions/GeometryException.h"
#include "Common/Types.h"
#include <algorithm>

namespace Topology3D
{

Topology3D::Topology3D(const std::unordered_map<std::string, Surface3D>& surfaces,
                       const std::unordered_map<std::string, Edge3D>& edges,
                       const std::unordered_map<std::string, Corner3D>& corners,
                       SeamCollection seams,
                       const std::unordered_map<std::string, Volume3D>& volumes) :
    surfaces_(surfaces),
    edges_(edges),
    corners_(corners),
    seams_(std::move(seams)),
    volumes_(volumes)
{
}

const Surface3D& Topology3D::getSurface(const std::string& id) const
{
    auto it = surfaces_.find(id);
    if (it == surfaces_.end())
    {
        OPENLOOM_THROW_ENTITY_NOT_FOUND("Surface", id);
    }
    return it->second;
}

const Edge3D& Topology3D::getEdge(const std::string& id) const
{
    auto it = edges_.find(id);
    if (it == edges_.end())
    {
        OPENLOOM_THROW_ENTITY_NOT_FOUND("Edge", id);
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

const SeamCollection& Topology3D::getSeamCollection() const
{
    return seams_;
}

} // namespace Topology3D