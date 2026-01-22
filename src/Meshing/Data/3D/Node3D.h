#pragma once
#include <string>
#include <vector>

#include "Common/Types.h"

namespace Meshing
{

/**
 * @brief A mesh node in 3D space
 *
 * Nodes are either interior nodes (no edge parameters) or boundary nodes
 * (with edge parameters and geometry IDs). Boundary nodes can be associated
 * with multiple edges/surfaces (e.g., corners where edges meet, or edges
 * where surfaces meet). The isBoundary() method returns true if the node
 * has any edge parameters.
 */
class Node3D
{
public:
    /// Construct an interior node
    explicit Node3D(const Point3D& coordinates);

    /// Construct a boundary node with edge parameters and geometry IDs
    Node3D(const Point3D& coordinates,
           const std::vector<double>& edgeParameters,
           const std::vector<std::string>& geometryIds);

    const Point3D& getCoordinates() const;
    void setCoordinates(const Point3D& coords);

    /// Returns true if this is a boundary node (has edge parameters)
    bool isBoundary() const;

    /// Get all geometry IDs associated with this node
    const std::vector<std::string>& getGeometryIds() const;

    /// Add a geometry ID to this node
    void addGeometryId(const std::string& id);

    /// Get the edge parameters for this boundary node
    const std::vector<double>& getEdgeParameters() const;

    /// Add an edge parameter to this node
    void addEdgeParameter(double param);

private:
    Point3D coordinates_;
    std::vector<double> edgeParameters_;
    std::vector<std::string> geometryIds_;
};

} // namespace Meshing