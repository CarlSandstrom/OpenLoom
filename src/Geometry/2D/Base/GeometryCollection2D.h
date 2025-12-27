#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "ICorner2D.h"
#include "IEdge2D.h"

namespace Geometry2D
{

/**
 * @brief Collection of 2D geometric entities in parametric space
 *
 * Holds corners and edges that define a 2D domain, typically extracted
 * from a 3D surface's parametric space.
 */
class GeometryCollection2D
{
public:
    GeometryCollection2D() = default;

    // Add entities
    void addCorner(std::unique_ptr<ICorner2D> corner);
    void addEdge(std::unique_ptr<IEdge2D> edge);

    // Access entities
    const ICorner2D* getCorner(const std::string& id) const;
    const IEdge2D* getEdge(const std::string& id) const;

    // Query all IDs
    std::vector<std::string> getAllCornerIds() const;
    std::vector<std::string> getAllEdgeIds() const;

    size_t getCornerCount() const { return corners_.size(); }
    size_t getEdgeCount() const { return edges_.size(); }

private:
    std::unordered_map<std::string, std::unique_ptr<ICorner2D>> corners_;
    std::unordered_map<std::string, std::unique_ptr<IEdge2D>> edges_;
};

} // namespace Geometry2D
