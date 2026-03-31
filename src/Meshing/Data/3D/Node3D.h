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

private:
    Point3D coordinates_;
};

} // namespace Meshing
