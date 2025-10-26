#include "GeometryManager.h"
#include <stdexcept>

using namespace Geometry;

GeometryManager::GeometryManager(std::unique_ptr<GeometryFactory> factory) :
    factory_(std::move(factory)) {}

void GeometryManager::addSurface(const std::string& id)
{
    surfaces_[id] = factory_->createSurface(id);
}

void GeometryManager::addEdge(const std::string& id)
{
    edges_[id] = factory_->createEdge(id);
}

void GeometryManager::addCorner(const std::string& id)
{
    corners_[id] = factory_->createCorner(id);
}

Surface* GeometryManager::getSurface(const std::string& id) const
{
    auto it = surfaces_.find(id);
    if (it == surfaces_.end())
    {
        throw std::runtime_error("Surface with ID '" + id + "' not found");
    }
    return it->second.get();
}

Edge* GeometryManager::getEdge(const std::string& id) const
{
    auto it = edges_.find(id);
    if (it == edges_.end())
    {
        throw std::runtime_error("Edge with ID '" + id + "' not found");
    }
    return it->second.get();
}

Corner* GeometryManager::getCorner(const std::string& id) const
{
    auto it = corners_.find(id);
    if (it == corners_.end())
    {
        throw std::runtime_error("Corner with ID '" + id + "' not found");
    }
    return it->second.get();
}

bool GeometryManager::hasSurface(const std::string& id) const
{
    return surfaces_.find(id) != surfaces_.end();
}

bool GeometryManager::hasEdge(const std::string& id) const
{
    return edges_.find(id) != edges_.end();
}

bool GeometryManager::hasCorner(const std::string& id) const
{
    return corners_.find(id) != corners_.end();
}

void GeometryManager::clear()
{
    surfaces_.clear();
    edges_.clear();
    corners_.clear();
}