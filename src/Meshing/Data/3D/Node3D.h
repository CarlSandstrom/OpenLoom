#pragma once

#include "Common/Types.h"

namespace Meshing
{

class Node3D
{
public:
    explicit Node3D(const Point3D& coordinates);

    const Point3D& getCoordinates() const;
    void setCoordinates(const Point3D& coords);

    /// The node's weight for regular-triangulation (weighted Delaunay)
    /// insertion -- a protecting ball's squared radius (OPE-176's
    /// crease-protection scheme), 0 for an ordinary (unweighted) node. See
    /// RegularPredicates3D for how this is used; a mesh where every node has
    /// weight 0 is mathematically indistinguishable from an ordinary
    /// Delaunay triangulation.
    double getWeight() const;
    void setWeight(double weight);

private:
    Point3D coordinates_;
    double weight_ = 0.0;
};

} // namespace Meshing
