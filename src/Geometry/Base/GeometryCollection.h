#pragma once

#include "Corner.h"
#include "Edge.h"
#include "Surface.h"
#include <memory>
#include <string>
#include <unordered_map>

namespace Geometry
{

/**
 * @brief Manager class that holds all geometry objects
 *
 * This class is geometry-engine agnostic and works with the abstract interfaces
 */
class GeometryCollection
{
public:
    explicit GeometryCollection(std::unordered_map<std::string, std::unique_ptr<Surface>> surfaces,
                                std::unordered_map<std::string, std::unique_ptr<Edge>> edges,
                                std::unordered_map<std::string, std::unique_ptr<Corner>> corners);

    // Retrieve geometry entities
    Surface* getSurface(const std::string& id) const;
    Edge* getEdge(const std::string& id) const;
    Corner* getCorner(const std::string& id) const;

private:
    std::unordered_map<std::string, std::unique_ptr<Surface>> surfaces_;
    std::unordered_map<std::string, std::unique_ptr<Edge>> edges_;
    std::unordered_map<std::string, std::unique_ptr<Corner>> corners_;
};

} // namespace Geometry