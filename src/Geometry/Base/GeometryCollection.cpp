#include "GeometryCollection.h"
#include <stdexcept>

using namespace Geometry;

GeometryCollection::GeometryCollection(std::unordered_map<std::string, std::unique_ptr<Surface>> surfaces,
                                       std::unordered_map<std::string, std::unique_ptr<Edge>> edges,
                                       std::unordered_map<std::string, std::unique_ptr<Corner>> corners) :
    surfaces_(std::move(surfaces)),
    edges_(std::move(edges)),
    corners_(std::move(corners))
{
}

Surface* GeometryCollection::getSurface(const std::string& id) const
{
    auto it = surfaces_.find(id);
    if (it == surfaces_.end())
    {
        throw std::runtime_error("Surface with ID '" + id + "' not found");
    }
    return it->second.get();
}

Edge* GeometryCollection::getEdge(const std::string& id) const
{
    auto it = edges_.find(id);
    if (it == edges_.end())
    {
        throw std::runtime_error("Edge with ID '" + id + "' not found");
    }
    return it->second.get();
}

Corner* GeometryCollection::getCorner(const std::string& id) const
{
    auto it = corners_.find(id);
    if (it == corners_.end())
    {
        throw std::runtime_error("Corner with ID '" + id + "' not found");
    }
    return it->second.get();
}
