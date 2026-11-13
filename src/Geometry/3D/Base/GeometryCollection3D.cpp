#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Common/Exceptions/GeometryException.h"

#include <utility>

namespace Geometry3D
{

GeometryCollection3D::GeometryCollection3D(std::unordered_map<std::string, std::unique_ptr<ISurface3D>> surfaces,
                                           std::unordered_map<std::string, std::unique_ptr<IEdge3D>> edges,
                                           std::unordered_map<std::string, std::unique_ptr<ICorner3D>> corners,
                                           std::unordered_map<std::string, std::unique_ptr<IVolume3D>> volumes) :
    surfaces_(std::move(surfaces)),
    edges_(std::move(edges)),
    corners_(std::move(corners)),
    volumes_(std::move(volumes))
{
}

ISurface3D* GeometryCollection3D::getSurface(const std::string& id) const
{
    auto entry = surfaces_.find(id);
    if (entry == surfaces_.end()) OPENLOOM_THROW_ENTITY_NOT_FOUND("Surface", id);
    return entry->second.get();
}

IEdge3D* GeometryCollection3D::getEdge(const std::string& id) const
{
    auto entry = edges_.find(id);
    if (entry == edges_.end()) OPENLOOM_THROW_ENTITY_NOT_FOUND("Edge", id);
    return entry->second.get();
}

ICorner3D* GeometryCollection3D::getCorner(const std::string& id) const
{
    auto entry = corners_.find(id);
    if (entry == corners_.end()) OPENLOOM_THROW_ENTITY_NOT_FOUND("Corner", id);
    return entry->second.get();
}

} // namespace Geometry3D
