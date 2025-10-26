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
 * @brief Abstract factory for creating geometry objects
 *
 * This provides the interface that concrete geometry engines (OpenCascade, etc.)
 * must implement to integrate with the mesher
 */
class GeometryFactory
{
public:
    virtual ~GeometryFactory() = default;

    /**
     * @brief Create a surface geometry object
     * @param id Unique identifier
     * @return Unique pointer to geometry surface
     */
    virtual std::unique_ptr<Surface> createSurface(const std::string& id) = 0;

    /**
     * @brief Create an edge geometry object
     * @param id Unique identifier
     * @return Unique pointer to geometry edge
     */
    virtual std::unique_ptr<Edge> createEdge(const std::string& id) = 0;

    /**
     * @brief Create a corner geometry object
     * @param id Unique identifier
     * @return Unique pointer to geometry corner
     */
    virtual std::unique_ptr<Corner> createCorner(const std::string& id) = 0;
};

/**
 * @brief Manager class that holds all geometry objects
 *
 * This class is geometry-engine agnostic and works with the abstract interfaces
 */
class GeometryManager
{
public:
    explicit GeometryManager(std::unique_ptr<GeometryFactory> factory);

    // Create and store geometry entities
    void addSurface(const std::string& id);
    void addEdge(const std::string& id);
    void addCorner(const std::string& id);

    // Retrieve geometry entities
    Surface* getSurface(const std::string& id) const;
    Edge* getEdge(const std::string& id) const;
    Corner* getCorner(const std::string& id) const;

    // Check existence
    bool hasSurface(const std::string& id) const;
    bool hasEdge(const std::string& id) const;
    bool hasCorner(const std::string& id) const;

    // Clear all geometry
    void clear();

private:
    std::unique_ptr<GeometryFactory> factory_;
    std::unordered_map<std::string, std::unique_ptr<Surface>> surfaces_;
    std::unordered_map<std::string, std::unique_ptr<Edge>> edges_;
    std::unordered_map<std::string, std::unique_ptr<Corner>> corners_;
};

} // namespace Geometry