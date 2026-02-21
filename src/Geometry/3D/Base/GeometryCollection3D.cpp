#include "GeometryCollection3D.h"
#include "Common/Exceptions/GeometryException.h"

namespace Geometry3D
{

GeometryCollection3D::GeometryCollection3D(std::unordered_map<std::string, std::unique_ptr<ISurface3D>> surfaces,
                                           std::unordered_map<std::string, std::unique_ptr<IEdge3D>> edges,
                                           std::unordered_map<std::string, std::unique_ptr<ICorner3D>> corners) :
    surfaces_(std::move(surfaces)),
    edges_(std::move(edges)),
    corners_(std::move(corners))
{
}

ISurface3D* GeometryCollection3D::getSurface(const std::string& id) const
{
    auto it = surfaces_.find(id);
    if (it == surfaces_.end())
    {
        OPENLOOM_THROW_ENTITY_NOT_FOUND("Surface", id);
    }
    return it->second.get();
}

IEdge3D* GeometryCollection3D::getEdge(const std::string& id) const
{
    auto it = edges_.find(id);
    if (it == edges_.end())
    {
        OPENLOOM_THROW_ENTITY_NOT_FOUND("Edge", id);
    }
    return it->second.get();
}

ICorner3D* GeometryCollection3D::getCorner(const std::string& id) const
{
    auto it = corners_.find(id);
    if (it == corners_.end())
    {
        OPENLOOM_THROW_ENTITY_NOT_FOUND("Corner", id);
    }
    return it->second.get();
}

} // namespace Geometry3D
