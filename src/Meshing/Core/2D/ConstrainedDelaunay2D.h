#pragma once

#include "Common/Types.h"
#include "Meshing/Data/MeshData2D.h"
#include "Meshing/Data/MeshData3D.h"
#include "Meshing/Data/TriangleElement.h"
#include <array>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Meshing
{

class MeshingContext2D;
class MeshMutator2D;
class MeshOperations2D;

class ConstrainedDelaunay2D
{
public:
    /**
     * @brief Construct 2D Delaunay triangulator with MeshingContext2D
     *
     * This constructor enables the generateConstrained() workflow,
     * similar to ConstrainedDelaunay3D.
     *
     * @param context The 2D meshing context containing geometry and topology
     */
    explicit ConstrainedDelaunay2D(MeshingContext2D& context, const std::vector<Point2D>& additionalPoints = {});

    ~ConstrainedDelaunay2D();

    std::vector<std::array<size_t, 3>> triangulate();

private:
    // Context (optional - for generateConstrained workflow)
    MeshingContext2D* context_ = nullptr;
    MeshData2D* meshData2D_ = nullptr;
    MeshMutator2D* operations_ = nullptr;
};

} // namespace Meshing
