#include "Meshing/Core/Delaunay3D.h"

#include "Meshing/Core/Computer.h"

#include <algorithm>
#include <limits>
#include <map>
#include <stdexcept>

namespace Meshing
{

Triangle::Triangle(int v0, int v1, int v2) :
    vertices{v0, v1, v2}
{
}

bool Triangle::operator==(const Triangle& other) const
{
    auto a = vertices;
    auto b = other.vertices;
    std::sort(a.begin(), a.end());
    std::sort(b.begin(), b.end());
    return a == b;
}

bool Triangle::operator<(const Triangle& other) const
{
    auto a = vertices;
    auto b = other.vertices;
    std::sort(a.begin(), a.end());
    std::sort(b.begin(), b.end());
    return a < b;
}

Tetrahedron::Tetrahedron() :
    vertices_{-1, -1, -1, -1},
    circumcenter_(Point3D::Zero()),
    circumradius_(0.0),
    isValid_(true)
{
}

Tetrahedron::Tetrahedron(int v0, int v1, int v2, int v3) :
    vertices_{v0, v1, v2, v3},
    circumcenter_(Point3D::Zero()),
    circumradius_(0.0),
    isValid_(true)
{
}

void Tetrahedron::computeCircumsphere(const std::vector<Point3D>& points)
{
    const Point3D& a = points[vertices_[0]];
    const Point3D& b = points[vertices_[1]];
    const Point3D& c = points[vertices_[2]];
    const Point3D& d = points[vertices_[3]];

    if (const auto sphere = Computer::getCircumscribingSphere(a, b, c, d))
    {
        circumcenter_ = sphere->center;
        circumradius_ = sphere->radius;
        isValid_ = true;
    }
    else
    {
        isValid_ = false;
    }
}

bool Tetrahedron::isInCircumsphere(const Point3D& p) const
{
    if (!isValid_)
    {
        return false;
    }

    ElementGeometry::CircumscribedSphere sphere{circumcenter_, circumradius_};
    return Computer::getIsPointInsideCircumscribingSphere(sphere, p);
}

std::array<Triangle, 4> Tetrahedron::getFaces() const
{
    return {
        Triangle(vertices_[0], vertices_[1], vertices_[2]),
        Triangle(vertices_[0], vertices_[1], vertices_[3]),
        Triangle(vertices_[0], vertices_[2], vertices_[3]),
        Triangle(vertices_[1], vertices_[2], vertices_[3])};
}

bool Tetrahedron::hasVertex(int vertexIndex) const
{
    return std::find(vertices_.begin(), vertices_.end(), vertexIndex) != vertices_.end();
}

double Tetrahedron::getShortestEdge(const std::vector<Point3D>& points) const
{
    double minLen = std::numeric_limits<double>::max();

    for (int i = 0; i < 4; ++i)
    {
        for (int j = i + 1; j < 4; ++j)
        {
            const double len = (points[vertices_[i]] - points[vertices_[j]]).norm();
            minLen = std::min(minLen, len);
        }
    }

    return minLen;
}

double Tetrahedron::getCircumradiusToShortestEdgeRatio(const std::vector<Point3D>& points) const
{
    const double shortestEdge = getShortestEdge(points);
    if (shortestEdge < 1e-15)
    {
        return std::numeric_limits<double>::max();
    }

    return circumradius_ / shortestEdge;
}

bool Tetrahedron::isSkinny(const std::vector<Point3D>& points, double threshold) const
{
    return getCircumradiusToShortestEdgeRatio(points) > threshold;
}

double Tetrahedron::volume(const std::vector<Point3D>& points) const
{
    return Computer::computeVolume(points[vertices_[0]],
                                   points[vertices_[1]],
                                   points[vertices_[2]],
                                   points[vertices_[3]]);
}

Delaunay3D::Delaunay3D(Meshing::MeshingContext& context) :
    context_(context),
    meshData_(context.getMeshData()),
    operations_(context.getOperations())
{
}

void Delaunay3D::initialize(const std::vector<Point3D>& points)
{
    vertices_.clear();
    tetrahedra_.clear();

    if (points.size() < 4)
    {
        throw std::runtime_error("Need at least 4 points for 3D triangulation");
    }

    createSuperTetrahedron(points);

    for (const auto& p : points)
    {
        insertVertex(p);
    }

    removeSuperTetrahedron();
}

void Delaunay3D::createSuperTetrahedron(const std::vector<Point3D>& points)
{
    double minX = std::numeric_limits<double>::max();
    double minY = std::numeric_limits<double>::max();
    double minZ = std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    double maxZ = -std::numeric_limits<double>::max();

    for (const auto& p : points)
    {
        minX = std::min(minX, p.x());
        minY = std::min(minY, p.y());
        minZ = std::min(minZ, p.z());
        maxX = std::max(maxX, p.x());
        maxY = std::max(maxY, p.y());
        maxZ = std::max(maxZ, p.z());
    }

    const double dx = maxX - minX;
    const double dy = maxY - minY;
    const double dz = maxZ - minZ;
    const double dmax = std::max({dx, dy, dz});

    const double midX = (minX + maxX) * 0.5;
    const double midY = (minY + maxY) * 0.5;
    const double midZ = (minZ + maxZ) * 0.5;

    const double scale = 10.0 * dmax;

    vertices_.push_back(Point3D(midX - scale, midY - scale, midZ - scale));
    vertices_.push_back(Point3D(midX + scale, midY - scale, midZ - scale));
    vertices_.push_back(Point3D(midX, midY + scale, midZ - scale));
    vertices_.push_back(Point3D(midX, midY, midZ + scale));

    Tetrahedron superTet(0, 1, 2, 3);
    superTet.computeCircumsphere(vertices_);
    tetrahedra_.push_back(superTet);
}

void Delaunay3D::insertVertex(const Point3D& point)
{
    const int vertexIndex = static_cast<int>(vertices_.size());
    vertices_.push_back(point);

    const std::vector<int> conflicting = findConflictingTetrahedra(point);
    if (conflicting.empty())
    {
        throw std::runtime_error("No conflicting tetrahedra found - point outside mesh?");
    }

    const std::vector<Triangle> boundary = findCavityBoundary(conflicting);

    for (int idx : conflicting)
    {
        tetrahedra_[idx].isValid_ = false;
    }

    retriangulate(vertexIndex, boundary);
}

std::vector<int> Delaunay3D::findConflictingTetrahedra(const Point3D& p) const
{
    std::vector<int> conflicting;

    for (size_t i = 0; i < tetrahedra_.size(); ++i)
    {
        if (!tetrahedra_[i].isValid_)
        {
            continue;
        }

        if (tetrahedra_[i].isInCircumsphere(p))
        {
            conflicting.push_back(static_cast<int>(i));
        }
    }

    return conflicting;
}

std::vector<Triangle> Delaunay3D::findCavityBoundary(const std::vector<int>& conflicting) const
{
    std::map<Triangle, int> faceCount;

    for (int tetIdx : conflicting)
    {
        const auto faces = tetrahedra_[tetIdx].getFaces();
        for (const auto& face : faces)
        {
            faceCount[face]++;
        }
    }

    std::vector<Triangle> boundary;
    boundary.reserve(faceCount.size());

    for (const auto& [face, count] : faceCount)
    {
        if (count == 1)
        {
            boundary.push_back(face);
        }
    }

    return boundary;
}

void Delaunay3D::retriangulate(int vertexIndex, const std::vector<Triangle>& boundary)
{
    for (const auto& face : boundary)
    {
        Tetrahedron newTet(vertexIndex, face.vertices[0], face.vertices[1], face.vertices[2]);
        newTet.computeCircumsphere(vertices_);
        tetrahedra_.push_back(newTet);
    }
}

void Delaunay3D::removeSuperTetrahedron()
{
    std::vector<Tetrahedron> validTets;
    validTets.reserve(tetrahedra_.size());

    for (auto tet : tetrahedra_)
    {
        if (!tet.isValid_)
        {
            continue;
        }

        bool hasSuperVertex = false;
        for (int i = 0; i < 4; ++i)
        {
            if (tet.vertices_[i] < 4)
            {
                hasSuperVertex = true;
                break;
            }
        }

        if (!hasSuperVertex)
        {
            for (int i = 0; i < 4; ++i)
            {
                tet.vertices_[i] -= 4;
            }
            validTets.push_back(tet);
        }
    }

    tetrahedra_ = std::move(validTets);

    if (vertices_.size() >= 4)
    {
        vertices_.erase(vertices_.begin(), vertices_.begin() + 4);
    }
}

double Delaunay3D::orient3D(const Point3D& a,
                            const Point3D& b,
                            const Point3D& c,
                            const Point3D& d) const
{
    const Point3D ba = b - a;
    const Point3D ca = c - a;
    const Point3D da = d - a;

    return ba.dot(ca.cross(da));
}

bool Delaunay3D::isDelaunay() const
{
    for (const auto& tet : tetrahedra_)
    {
        if (!tet.isValid_)
        {
            continue;
        }

        for (size_t i = 0; i < vertices_.size(); ++i)
        {
            if (tet.hasVertex(static_cast<int>(i)))
            {
                continue;
            }

            if (tet.isInCircumsphere(vertices_[i]))
            {
                return false;
            }
        }
    }

    return true;
}

} // namespace Meshing
