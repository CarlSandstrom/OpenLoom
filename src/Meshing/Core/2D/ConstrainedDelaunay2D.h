#pragma once

#include "Common/Types.h"
#include "Geometry/2D/Base/DiscretizationSettings2D.h"
#include "Meshing/Core/2D/GeometryStructures2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include <array>
#include <memory>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace Geometry2D
{
class IFace2D;
}

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
     * @param discretizationSettings Settings for discretizing geometry (optional)
     * @param additionalPoints Additional points to include in triangulation (optional)
     */
    explicit ConstrainedDelaunay2D(MeshingContext2D& context,
                                   const Geometry2D::DiscretizationSettings2D& discretizationSettings = {},
                                   const std::vector<Point2D>& additionalPoints = {});

    ~ConstrainedDelaunay2D();

    void triangulate();
    std::vector<ConstrainedSegment2D> getConstrainedEdges() const;

private:
    void exportAndVerifyMesh();
    void removeHoleTriangles();

private:
    size_t exportCounter_ = 0;

    std::vector<ConstrainedSegment2D> constrainedEdges_;
    MeshingContext2D* context_ = nullptr;
    MeshData2D* meshData2D_ = nullptr;
    MeshMutator2D* meshMutator_ = nullptr;
    MeshOperations2D* meshOperations_ = nullptr;
};

} // namespace Meshing
