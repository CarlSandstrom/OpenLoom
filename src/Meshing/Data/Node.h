#pragma once
#include <string>

#include "Common/Types.h"

namespace Meshing
{

class Node
{
public:
    explicit Node(const Point3D& coordinates);

    const Point3D& getCoordinates() const;
    void setCoordinates(const Point3D& coords);

    // Boundary marking
    bool isBoundary() const;
    void setBoundary(bool boundary);

    // Optional: geometry entity association
    void setGeometryId(const std::string& id);
    const std::string& getGeometryId() const;

private:
    Point3D coordinates_;
    bool isBoundary_;
    std::string geometryId_; // Links to topology entities
};

} // namespace Meshing