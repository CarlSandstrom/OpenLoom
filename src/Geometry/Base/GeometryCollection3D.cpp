#include "GeometryCollection3D.h"
#include <stdexcept>

namespace Geometry3D
{

GeometryCollection3D::GeometryCollection3D(std::unordered_map<std::string, std::unique_ptr<Surface3D>> surfaces,
                                           std::unordered_map<std::string, std::unique_ptr<Edge3D>> edges,
                                           std::unordered_map<std::string, std::unique_ptr<Corner3D>> corners) :
    surfaces_(std::move(surfaces)),
    edges_(std::move(edges)),
    corners_(std::move(corners))
{
}

Surface3D* GeometryCollection3D::getSurface(const std::string& id) const
{
    auto it = surfaces_.find(id);
    if (it == surfaces_.end())
    {
        throw std::runtime_error("Surface with ID '" + id + "' not found");
    }
    return it->second.get();
}

Edge3D* GeometryCollection3D::getEdge(const std::string& id) const
{
    auto it = edges_.find(id);
    if (it == edges_.end())
    {
        throw std::runtime_error("Edge with ID '" + id + "' not found");
    }
    return it->second.get();
}

Corner3D* GeometryCollection3D::getCorner(const std::string& id) const
{
    auto it = corners_.find(id);
    if (it == corners_.end())
    {
        throw std::runtime_error("Corner with ID '" + id + "' not found");
    }
    return it->second.get();
}

} // namespace Geometry3D
