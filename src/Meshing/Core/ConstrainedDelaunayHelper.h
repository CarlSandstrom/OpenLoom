#pragma once

#include <array>
#include <unordered_set>
#include <vector>

#include "Common/Types.h"
#include "Meshing/Core/ConstraintStructures.h"

namespace Meshing
{

class ConstrainedDelaunay3D;
class MeshOperations;
class MeshData;
class TetrahedralElement;

class ConstrainedDelaunay3DHelper
{
public:
    static std::pair<size_t, size_t> makeSegmentKey(size_t a, size_t b);
    static std::array<size_t, 3> makeTriangleKey(size_t a, size_t b, size_t c);

    static bool segmentExists(const ConstrainedDelaunay3D& delaunay,
                              size_t nodeId1,
                              size_t nodeId2,
                              const std::unordered_set<size_t>& activeTetrahedra,
                              const std::unordered_set<std::pair<size_t, size_t>, PairHash>& satisfiedSegments);

    static bool facetExists(const ConstrainedDelaunay3D& delaunay,
                            size_t n0,
                            size_t n1,
                            size_t n2,
                            const std::unordered_set<size_t>& activeTetrahedra,
                            const std::unordered_set<std::array<size_t, 3>, TriangleHash>& satisfiedFacets);

    static std::vector<size_t> findIntersectingTetrahedra(const ConstrainedDelaunay3D& delaunay,
                                                          const MeshData& meshData,
                                                          size_t nodeId1,
                                                          size_t nodeId2,
                                                          const std::unordered_set<size_t>& activeTetrahedra);

    static std::vector<size_t> findIntersectingTetrahedraForFacet(const ConstrainedDelaunay3D& delaunay,
                                                                  const MeshData& meshData,
                                                                  size_t n0,
                                                                  size_t n1,
                                                                  size_t n2,
                                                                  const std::unordered_set<size_t>& activeTetrahedra);

    static void retriangulateCavityWithSegment(MeshOperations& operations,
                                               std::unordered_set<size_t>& activeTetrahedra,
                                               size_t n1,
                                               size_t n2,
                                               const std::vector<std::array<size_t, 3>>& boundary);

    static void retriangulateCavityWithFacet(MeshOperations& operations,
                                             std::unordered_set<size_t>& activeTetrahedra,
                                             size_t n0,
                                             size_t n1,
                                             size_t n2,
                                             const std::vector<std::array<size_t, 3>>& boundary);

private:
    static bool segmentIntersectsTet(const MeshData& meshData,
                                     const Point3D& p1,
                                     const Point3D& p2,
                                     const TetrahedralElement& tet);

    static bool triangleIntersectsTet(const MeshData& meshData,
                                      const Point3D& t0,
                                      const Point3D& t1,
                                      const Point3D& t2,
                                      const TetrahedralElement& tet);
};

} // namespace Meshing
