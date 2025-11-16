#pragma once

#include <algorithm>
#include <array>
#include <vector>

#include "Common/Types.h"
#include "Meshing/Core/MeshingContext.h"
#include "Meshing/Data/MeshData.h"
#include "Meshing/Data/MeshOperations.h"

namespace Meshing
{

struct Triangle
{
    std::array<int, 3> vertices;

    Triangle(int v0 = -1, int v1 = -1, int v2 = -1);

    bool operator==(const Triangle& other) const;
    bool operator<(const Triangle& other) const;
};

struct Tetrahedron
{
    std::array<int, 4> vertices_;
    Point3D circumcenter_;
    double circumradius_;
    bool isValid_;

    Tetrahedron();
    Tetrahedron(int v0, int v1, int v2, int v3);

    void computeCircumsphere(const std::vector<Point3D>& points);
    bool isInCircumsphere(const Point3D& p) const;
    std::array<Triangle, 4> getFaces() const;
    bool hasVertex(int vertexIndex) const;
    double getShortestEdge(const std::vector<Point3D>& points) const;
    double getCircumradiusToShortestEdgeRatio(const std::vector<Point3D>& points) const;
    bool isSkinny(const std::vector<Point3D>& points, double threshold = 2.0) const;
    double volume(const std::vector<Point3D>& points) const;
};

class Delaunay3D
{
public:
    Delaunay3D(Meshing::MeshingContext& context);
    ~Delaunay3D() = default;

    void initialize(const std::vector<Point3D>& points);
    void insertVertex(const Point3D& point);

    const std::vector<Point3D>& getVertices() const { return vertices_; }
    const std::vector<Tetrahedron>& getTetrahedra() const { return tetrahedra_; }

    bool isDelaunay() const;

private:
    void createSuperTetrahedron(const std::vector<Point3D>& points);
    void removeSuperTetrahedron();
    std::vector<int> findConflictingTetrahedra(const Point3D& p) const;
    std::vector<Triangle> findCavityBoundary(const std::vector<int>& conflicting) const;
    void retriangulate(int vertexIndex, const std::vector<Triangle>& boundary);
    double orient3D(const Point3D& a,
                    const Point3D& b,
                    const Point3D& c,
                    const Point3D& d) const;

    std::vector<Point3D> vertices_;
    std::vector<Tetrahedron> tetrahedra_;

    Meshing::MeshingContext& context_;
    Meshing::MeshData& meshData_;
    Meshing::MeshOperations& operations_;
};

} // namespace Meshing
