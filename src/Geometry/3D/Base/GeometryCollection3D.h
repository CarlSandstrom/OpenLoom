#pragma once

#include "ICorner3D.h"
#include "IEdge3D.h"
#include "ISurface3D.h"
#include "IVolume3D.h"
#include <memory>
#include <string>
#include <unordered_map>

namespace Geometry3D
{

/**
 * @brief Manager class that holds all geometry objects
 *
 * This class is geometry-engine agnostic and works with the abstract interfaces
 */
class GeometryCollection3D
{
public:
    explicit GeometryCollection3D(std::unordered_map<std::string, std::unique_ptr<ISurface3D>> surfaces,
                                  std::unordered_map<std::string, std::unique_ptr<IEdge3D>> edges,
                                  std::unordered_map<std::string, std::unique_ptr<ICorner3D>> corners,
                                  std::unordered_map<std::string, std::unique_ptr<IVolume3D>> volumes = {});

    // Retrieve geometry entities
    ISurface3D* getSurface(const std::string& id) const;
    IEdge3D* getEdge(const std::string& id) const;
    ICorner3D* getCorner(const std::string& id) const;
    IVolume3D* getVolume(const std::string& id) const;

private:
    std::unordered_map<std::string, std::unique_ptr<ISurface3D>> surfaces_;
    std::unordered_map<std::string, std::unique_ptr<IEdge3D>> edges_;
    std::unordered_map<std::string, std::unique_ptr<ICorner3D>> corners_;
    std::unordered_map<std::string, std::unique_ptr<IVolume3D>> volumes_;
};

} // namespace Geometry3D