#pragma once

#include "Geometry/2D/Base/IFace2D.h"
#include <cstddef>

namespace Meshing
{

class MeshData2D;
class MeshMutator2D;

class HoleTriangleRemover
{
public:
    HoleTriangleRemover(MeshData2D& meshData,
                        MeshMutator2D& mutator,
                        const Geometry2D::IFace2D& face);

    void removeInvalidTriangles();
    size_t getRemovedCount() const { return removedCount_; }

private:
    Point2D computeCentroid(size_t elemId) const;

    MeshData2D& meshData_;
    MeshMutator2D& mutator_;
    const Geometry2D::IFace2D& face_;
    size_t removedCount_;
};

} // namespace Meshing
