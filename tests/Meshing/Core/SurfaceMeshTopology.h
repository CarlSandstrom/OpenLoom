#pragma once

#include <cstddef>
#include <map>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace Meshing
{
struct SurfaceMesh3D;
} // namespace Meshing

namespace TestSupport
{

using SurfaceMeshEdge = std::pair<size_t, size_t>;

/**
 * @brief The triangle-level topology of an exported surface mesh: which
 * vertices it uses, and how many triangles meet along each edge.
 *
 * Exists so the end-to-end mesher tests can assert that the mesh of a closed
 * solid really is a closed surface. Nothing else in the test suite did:
 * before this, only the BoxWithHole volume test checked watertightness and
 * only the torus checked an Euler characteristic, so a puncture in the
 * cylinder, sphere or box surface mesh passed every test they had.
 *
 * Deliberately not reusing RestrictedFaceAudit::findNonManifoldEdges(). That
 * answers a related but different question: it reads the expected per-edge
 * count off the CAD topology (so it tolerates a legitimate triple line), and
 * it inspects the mesher's internal restricted-face set. A test of the
 * OUTPUT has to count the output.
 */
struct SurfaceMeshTopology
{
    std::unordered_set<size_t> vertices;
    std::map<SurfaceMeshEdge, size_t> trianglesPerEdge;
    size_t triangleCount = 0;

    /// V - E + F: 2 for a topological sphere, 0 for genus 1, 2 - 2g in
    /// general.
    long long eulerCharacteristic() const;

    /// Every edge not shared by exactly expectedCount triangles -- fewer is a
    /// hole, more is a duplicated patch.
    std::vector<SurfaceMeshEdge> edgesNotSharedBy(size_t expectedCount) const;

    /// The first few offending edges with their counts, for a failure
    /// message.
    std::string describe(const std::vector<SurfaceMeshEdge>& edges) const;
};

SurfaceMeshTopology computeSurfaceMeshTopology(const Meshing::SurfaceMesh3D& surfaceMesh);

} // namespace TestSupport
