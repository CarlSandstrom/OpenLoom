#pragma once

#include <array>
#include <unordered_set>
#include <vector>

#include "Common/Types.h"
#include "Meshing/Core/Computer.h"
#include "Meshing/Core/MeshingContext.h"
#include "Meshing/Data/MeshData.h"
#include "Meshing/Data/MeshOperations.h"
#include "Meshing/Data/TetrahedralElement.h"

namespace Meshing
{

class Delaunay3D
{
public:
    Delaunay3D(Meshing::MeshingContext& context);
    ~Delaunay3D() = default;

    void initialize(const std::vector<Point3D>& points);
    void insertVertex(const Point3D& point);

    const MeshData& getMeshData() const { return meshData_; }
    MeshData& getMeshData() { return meshData_; }

    const std::unordered_set<size_t>& getActiveTetrahedronIds() const { return activeTetrahedra_; }
    bool isElementActive(size_t elementId) const;

    Computer& getComputer() { return computer_; }
    const Computer& getComputer() const { return computer_; }

    const TetrahedralElement* getTetrahedralElement(size_t elementId) const;

    bool isDelaunay() const;

private:
    void createSuperTetrahedron(const std::vector<Point3D>& points);
    void removeSuperTetrahedron();
    std::vector<size_t> findConflictingTetrahedra(const Point3D& p) const;
    std::vector<std::array<size_t, 3>> findCavityBoundary(const std::vector<size_t>& conflicting) const;
    void retriangulate(size_t vertexNodeId, const std::vector<std::array<size_t, 3>>& boundary);

    Computer computer_;
    std::vector<size_t> superNodeIds_;
    std::unordered_set<size_t> activeTetrahedra_;

    Meshing::MeshingContext& context_;
    Meshing::MeshData& meshData_;
    Meshing::MeshOperations& operations_;
};

} // namespace Meshing
