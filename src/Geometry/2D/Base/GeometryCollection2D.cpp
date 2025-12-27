#include "GeometryCollection2D.h"

namespace Geometry2D
{

void GeometryCollection2D::addCorner(std::unique_ptr<ICorner2D> corner)
{
    if (corner)
    {
        std::string id = corner->getId();
        corners_[id] = std::move(corner);
    }
}

void GeometryCollection2D::addEdge(std::unique_ptr<IEdge2D> edge)
{
    if (edge)
    {
        std::string id = edge->getId();
        edges_[id] = std::move(edge);
    }
}

const ICorner2D* GeometryCollection2D::getCorner(const std::string& id) const
{
    auto it = corners_.find(id);
    return it != corners_.end() ? it->second.get() : nullptr;
}

const IEdge2D* GeometryCollection2D::getEdge(const std::string& id) const
{
    auto it = edges_.find(id);
    return it != edges_.end() ? it->second.get() : nullptr;
}

std::vector<std::string> GeometryCollection2D::getAllCornerIds() const
{
    std::vector<std::string> ids;
    ids.reserve(corners_.size());
    for (const auto& [id, _] : corners_)
    {
        ids.push_back(id);
    }
    return ids;
}

std::vector<std::string> GeometryCollection2D::getAllEdgeIds() const
{
    std::vector<std::string> ids;
    ids.reserve(edges_.size());
    for (const auto& [id, _] : edges_)
    {
        ids.push_back(id);
    }
    return ids;
}

} // namespace Geometry2D
