#pragma once

#include "Corner3D.h"
#include "Edge3D.h"
#include "Surface3D.h"
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
                                  std::unordered_map<std::string, std::unique_ptr<ICorner3D>> corners);

    // Retrieve geometry entities
    ISurface3D* getSurface(const std::string& id) const;
    IEdge3D* getEdge(const std::string& id) const;
    ICorner3D* getCorner(const std::string& id) const;

private:
    std::unordered_map<std::string, std::unique_ptr<ISurface3D>> surfaces_;
    std::unordered_map<std::string, std::unique_ptr<IEdge3D>> edges_;
    std::unordered_map<std::string, std::unique_ptr<ICorner3D>> corners_;
};

} // namespace Geometry3D