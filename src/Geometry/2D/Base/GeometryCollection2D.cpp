#include "Geometry/2D/Base/GeometryCollection2D.h"

#include <utility>

namespace Geometry2D
{

void GeometryCollection2D::addCorner(std::unique_ptr<ICorner2D> corner)
{
    if (!corner) return;
    auto id = corner->getId();
    corners_[id] = std::move(corner);
}

void GeometryCollection2D::addEdge(std::unique_ptr<IEdge2D> edge)
{
    if (!edge) return;
    auto id = edge->getId();
    edges_[id] = std::move(edge);
}

const ICorner2D* GeometryCollection2D::getCorner(const std::string& id) const
{
    auto entry = corners_.find(id);
    return entry != corners_.end() ? entry->second.get() : nullptr;
}

const IEdge2D* GeometryCollection2D::getEdge(const std::string& id) const
{
    auto entry = edges_.find(id);
    return entry != edges_.end() ? entry->second.get() : nullptr;
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

} // namespace Geometry2D
