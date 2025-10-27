#pragma once
#include <array>
#include <string>

namespace Meshing
{

class Node
{
public:
    Node(size_t id, const std::array<double, 3>& coordinates);

    size_t getId() const;
    const std::array<double, 3>& getCoordinates() const;
    void setCoordinates(const std::array<double, 3>& coords);

    // Boundary marking
    bool isBoundary() const;
    void setBoundary(bool boundary);

    // Optional: geometry entity association
    void setGeometryId(const std::string& id);
    const std::string& getGeometryId() const;

private:
    size_t id_;
    std::array<double, 3> coordinates_;
    bool isBoundary_;
    std::string geometryId_; // Links to topology entities
};

} // namespace Meshing