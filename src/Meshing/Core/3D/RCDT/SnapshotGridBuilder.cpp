#include "Meshing/Core/3D/RCDT/SnapshotGridBuilder.h"

#include "Export/VtkGrid.h"
#include "Meshing/Data/3D/SurfaceMesh3D.h"

#include <numeric>
#include <string>
#include <vector>

namespace Meshing
{

namespace
{
// std::map orders the surface ids, so an index means the same surface across runs.
std::vector<int> triangleSurfaceIndices(const SurfaceMesh3D& surfaceMesh);
} // namespace

Export::VtkGrid SnapshotGridBuilder::fromSurfaceMesh(const SurfaceMesh3D& surfaceMesh)
{
    Export::VtkGrid grid;
    grid.points = surfaceMesh.nodes;
    grid.nodeIds.resize(surfaceMesh.nodes.size());
    std::iota(grid.nodeIds.begin(), grid.nodeIds.end(), std::size_t{0});

    for (auto& triangle : surfaceMesh.triangles)
        grid.cells.push_back({Export::VtkCellType::Triangle, {triangle[0], triangle[1], triangle[2]}});
    grid.cellFields.push_back({"SurfaceID", triangleSurfaceIndices(surfaceMesh)});
    return grid;
}

namespace
{

std::vector<int> triangleSurfaceIndices(const SurfaceMesh3D& surfaceMesh)
{
    std::vector<int> indices(surfaceMesh.triangles.size(), -1);
    int surfaceIndex = 0;
    for (auto& [surfaceId, triangleIds] : surfaceMesh.faceTriangleIds)
    {
        for (auto triangleId : triangleIds)
            indices[triangleId] = surfaceIndex;
        ++surfaceIndex;
    }
    return indices;
}

} // namespace

} // namespace Meshing
