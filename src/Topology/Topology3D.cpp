#include "Topology/Topology3D.h"
#include "Common/Exceptions/GeometryException.h"
#include <utility>

namespace
{

template <typename Entity>
std::vector<std::string> collectIds(const std::unordered_map<std::string, Entity>& entities);

} // namespace

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
    if (it == surfaces_.end()) OPENLOOM_THROW_ENTITY_NOT_FOUND("Surface", id);
    return it->second;
}

const Edge3D& Topology3D::getEdge(const std::string& id) const
{
    auto it = edges_.find(id);
    if (it == edges_.end()) OPENLOOM_THROW_ENTITY_NOT_FOUND("Edge", id);
    return it->second;
}

std::vector<std::string> Topology3D::getAllSurfaceIds() const
{
    return collectIds(surfaces_);
}

std::vector<std::string> Topology3D::getAllEdgeIds() const
{
    return collectIds(edges_);
}

std::vector<std::string> Topology3D::getAllCornerIds() const
{
    return collectIds(corners_);
}

const SeamCollection& Topology3D::getSeamCollection() const
{
    return seams_;
}

} // namespace Topology3D

namespace
{

template <typename Entity>
std::vector<std::string> collectIds(const std::unordered_map<std::string, Entity>& entities)
{
    std::vector<std::string> ids;
    ids.reserve(entities.size());
    for (auto& [id, entity] : entities)
    {
        ids.push_back(id);
    }
    return ids;
}

} // namespace
