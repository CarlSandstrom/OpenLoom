#pragma once

#include "Common/Types.h"
#include "DiscretizationResult2D.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/2D/TriangleElement.h"
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace Meshing
{

class MeshingContext2D;
class MeshOperations2D;

class ConstrainedDelaunay2D
{
public:
    /**
     * @brief Construct 2D Delaunay triangulator with MeshingContext2D
     *
     * @param context The 2D meshing context containing geometry and topology
     * @param discretization Pre-computed edge discretization from EdgeDiscretizer2D
     * @param additionalPoints Additional points to include in triangulation (optional)
     * @param debugExportFilenamePrefix Prefix for debug export filenames (optional)
     */
    explicit ConstrainedDelaunay2D(MeshingContext2D& context,
                                   const DiscretizationResult2D& discretization,
                                   const std::vector<Point2D>& additionalPoints = {},
                                   const std::string& debugExportFilenamePrefix = "constrained_delaunay");

    ~ConstrainedDelaunay2D() = default;

    void triangulate();

    const std::map<size_t, size_t>& getPointIndexToNodeIdMap() const { return pointIndexToNodeIdMap_; }

private:
    size_t exportCounter_ = 0;
    std::string debugExportFilenamePrefix_;

    std::map<size_t, size_t> pointIndexToNodeIdMap_;
    DiscretizationResult2D discretization_;
    std::vector<Point2D> additionalPoints_;

    // Non-owning pointers, obtained from context_
    MeshingContext2D* context_ = nullptr;
    MeshData2D* meshData2D_ = nullptr;
    MeshOperations2D* meshOperations_ = nullptr;
};

} // namespace Meshing
