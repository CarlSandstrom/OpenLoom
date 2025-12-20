#include "ConstrainedDelaunayHelper.h"

#include <Eigen/Dense>
#include <algorithm>
#include <memory>

#include "Meshing/Core/3D/ConstrainedDelaunay3D.h"
#include "Meshing/Data/MeshData3D.h"
#include "Meshing/Data/MeshMutator3D.h"
#include "Meshing/Data/TetrahedralElement.h"

namespace Meshing
{

std::pair<size_t, size_t> ConstrainedDelaunay3DHelper::makeSegmentKey(size_t a, size_t b)
{
    return (a < b) ? std::make_pair(a, b) : std::make_pair(b, a);
}

std::array<size_t, 3> ConstrainedDelaunay3DHelper::makeTriangleKey(size_t a, size_t b, size_t c)
{
    std::array<size_t, 3> key = {a, b, c};
    std::sort(key.begin(), key.end());
    return key;
}

bool ConstrainedDelaunay3DHelper::segmentExists(const ConstrainedDelaunay3D& delaunay,
                                                size_t nodeId1,
                                                size_t nodeId2,
                                                const std::unordered_set<size_t>& activeTetrahedra,
                                                const std::unordered_set<std::pair<size_t, size_t>, PairHash>& satisfiedSegments)
{
    const auto key = makeSegmentKey(nodeId1, nodeId2);
    if (satisfiedSegments.count(key) > 0)
    {
        return true;
    }

    for (size_t tetId : activeTetrahedra)
    {
        const auto* tet = delaunay.getTetrahedralElement(tetId);
        if (tet == nullptr)
        {
            continue;
        }

        for (size_t i = 0; i < 6; ++i)
        {
            const auto edge = tet->getEdge(i);
            if (makeSegmentKey(edge[0], edge[1]) == key)
            {
                return true;
            }
        }
    }

    return false;
}

bool ConstrainedDelaunay3DHelper::facetExists(const ConstrainedDelaunay3D& delaunay,
                                              size_t n0,
                                              size_t n1,
                                              size_t n2,
                                              const std::unordered_set<size_t>& activeTetrahedra,
                                              const std::unordered_set<std::array<size_t, 3>, TriangleHash>& satisfiedFacets)
{
    const auto key = makeTriangleKey(n0, n1, n2);
    if (satisfiedFacets.count(key) > 0)
    {
        return true;
    }

    for (size_t tetId : activeTetrahedra)
    {
        const auto* tet = delaunay.getTetrahedralElement(tetId);
        if (tet == nullptr)
        {
            continue;
        }

        for (size_t i = 0; i < 4; ++i)
        {
            const auto face = tet->getFace(i);
            if (makeTriangleKey(face[0], face[1], face[2]) == key)
            {
                return true;
            }
        }
    }

    return false;
}

std::vector<size_t> ConstrainedDelaunay3DHelper::findIntersectingTetrahedra(
    const ConstrainedDelaunay3D& delaunay,
    const MeshData3D& meshData,
    size_t nodeId1,
    size_t nodeId2,
    const std::unordered_set<size_t>& activeTetrahedra)
{
    const Point3D& p1 = meshData.getNode(nodeId1)->getCoordinates();
    const Point3D& p2 = meshData.getNode(nodeId2)->getCoordinates();

    std::vector<size_t> result;
    for (size_t tetId : activeTetrahedra)
    {
        const auto* tet = delaunay.getTetrahedralElement(tetId);
        if (tet == nullptr || (tet->hasNode(nodeId1) && tet->hasNode(nodeId2)))
        {
            continue;
        }

        if (segmentIntersectsTet(meshData, p1, p2, *tet))
        {
            result.push_back(tetId);
        }
    }

    return result;
}

std::vector<size_t> ConstrainedDelaunay3DHelper::findIntersectingTetrahedraForFacet(
    const ConstrainedDelaunay3D& delaunay,
    const MeshData3D& meshData,
    size_t n0,
    size_t n1,
    size_t n2,
    const std::unordered_set<size_t>& activeTetrahedra)
{
    const Point3D& t0 = meshData.getNode(n0)->getCoordinates();
    const Point3D& t1 = meshData.getNode(n1)->getCoordinates();
    const Point3D& t2 = meshData.getNode(n2)->getCoordinates();

    std::vector<size_t> result;
    for (size_t tetId : activeTetrahedra)
    {
        const auto* tet = delaunay.getTetrahedralElement(tetId);
        if (tet == nullptr || tet->containsAll({n0, n1, n2}))
        {
            continue;
        }

        if (triangleIntersectsTet(meshData, t0, t1, t2, *tet))
        {
            result.push_back(tetId);
        }
    }

    return result;
}

void ConstrainedDelaunay3DHelper::retriangulateCavityWithSegment(
    MeshMutator3D& mutator,
    std::unordered_set<size_t>& activeTetrahedra,
    size_t n1,
    size_t n2,
    const std::vector<std::array<size_t, 3>>& boundary)
{
    for (const auto& face : boundary)
    {
        const bool has1 = (face[0] == n1 || face[1] == n1 || face[2] == n1);
        const bool has2 = (face[0] == n2 || face[1] == n2 || face[2] == n2);

        if (has1 && has2)
        {
            continue;
        }

        std::array<size_t, 4> nodes;
        if (has1)
        {
            nodes = {n2, face[0], face[1], face[2]};
        }
        else if (has2)
        {
            nodes = {n1, face[0], face[1], face[2]};
        }
        else
        {
            nodes = {n1, n2, face[0], face[1]};
        }

        auto element = std::make_unique<TetrahedralElement>(nodes);
        activeTetrahedra.insert(mutator.addElement(std::move(element)));
    }
}

void ConstrainedDelaunay3DHelper::retriangulateCavityWithFacet(
    MeshMutator3D& mutator,
    std::unordered_set<size_t>& activeTetrahedra,
    size_t n0,
    size_t n1,
    size_t n2,
    const std::vector<std::array<size_t, 3>>& boundary)
{
    for (const auto& face : boundary)
    {
        const bool hasAll = (face[0] == n0 || face[0] == n1 || face[0] == n2) &&
                            (face[1] == n0 || face[1] == n1 || face[1] == n2) &&
                            (face[2] == n0 || face[2] == n1 || face[2] == n2);

        if (hasAll)
        {
            continue;
        }

        const std::array<size_t, 3> constraintNodes = {n0, n1, n2};
        for (size_t cn : constraintNodes)
        {
            if (face[0] != cn && face[1] != cn && face[2] != cn)
            {
                std::array<size_t, 4> nodes = {cn, face[0], face[1], face[2]};
                auto element = std::make_unique<TetrahedralElement>(nodes);
                activeTetrahedra.insert(mutator.addElement(std::move(element)));
                break;
            }
        }
    }
}

bool ConstrainedDelaunay3DHelper::segmentIntersectsTet(const MeshData3D& meshData,
                                                       const Point3D& p1,
                                                       const Point3D& p2,
                                                       const TetrahedralElement& tet)
{
    const auto& ids = tet.getNodeIds();
    const Point3D& v0 = meshData.getNode(ids[0])->getCoordinates();
    const Point3D& v1 = meshData.getNode(ids[1])->getCoordinates();
    const Point3D& v2 = meshData.getNode(ids[2])->getCoordinates();
    const Point3D& v3 = meshData.getNode(ids[3])->getCoordinates();

    auto pointInTet = [&](const Point3D& p) {
        Eigen::Matrix4d A;
        A << v0.x(), v1.x(), v2.x(), v3.x(),
            v0.y(), v1.y(), v2.y(), v3.y(),
            v0.z(), v1.z(), v2.z(), v3.z(),
            1.0, 1.0, 1.0, 1.0;

        Eigen::Vector4d b(p.x(), p.y(), p.z(), 1.0);
        Eigen::Vector4d bary = A.colPivHouseholderQr().solve(b);
        return (bary.array() >= -1e-10).all();
    };

    return pointInTet(p1) || pointInTet(p2);
}

bool ConstrainedDelaunay3DHelper::triangleIntersectsTet(const MeshData3D& meshData,
                                                        const Point3D& t0,
                                                        const Point3D& t1,
                                                        const Point3D& t2,
                                                        const TetrahedralElement& tet)
{
    const auto& ids = tet.getNodeIds();
    const Point3D& v0 = meshData.getNode(ids[0])->getCoordinates();
    const Point3D& v1 = meshData.getNode(ids[1])->getCoordinates();
    const Point3D& v2 = meshData.getNode(ids[2])->getCoordinates();
    const Point3D& v3 = meshData.getNode(ids[3])->getCoordinates();

    auto pointInTet = [&](const Point3D& p) {
        Eigen::Matrix4d A;
        A << v0.x(), v1.x(), v2.x(), v3.x(),
            v0.y(), v1.y(), v2.y(), v3.y(),
            v0.z(), v1.z(), v2.z(), v3.z(),
            1.0, 1.0, 1.0, 1.0;

        Eigen::Vector4d b(p.x(), p.y(), p.z(), 1.0);
        Eigen::Vector4d bary = A.colPivHouseholderQr().solve(b);
        return (bary.array() >= -1e-10).all();
    };

    if (pointInTet(t0) || pointInTet(t1) || pointInTet(t2))
    {
        return true;
    }

    Vector3D normal = (t1 - t0).cross(t2 - t0);
    if (normal.norm() < 1e-10)
    {
        return false;
    }
    normal.normalize();

    const double d = -normal.dot(t0);
    auto dist = [&](const Point3D& p) { return normal.dot(p) + d; };

    const double d0 = dist(v0);
    const double d1 = dist(v1);
    const double d2 = dist(v2);
    const double d3 = dist(v3);

    return !((d0 > 0 && d1 > 0 && d2 > 0 && d3 > 0) ||
             (d0 < 0 && d1 < 0 && d2 < 0 && d3 < 0));
}

} // namespace Meshing
