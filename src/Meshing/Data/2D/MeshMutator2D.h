#pragma once

#include "Common/Types.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/Base/IElement.h"
#include "Meshing/Data/CurveSegmentManager.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace Meshing
{

class MeshMutator2D
{
public:
    explicit MeshMutator2D(MeshData2D& meshData2D);

    size_t addNode(const Point2D& coordinates);
    size_t addBoundaryNode(const Point2D& coordinates, const std::vector<std::string>& geometryIds);

    size_t addElement(std::unique_ptr<IElement> element);
    void removeElement(size_t id);

    void setCurveSegmentManager(CurveSegmentManager curveSegmentManager);
    std::pair<size_t, size_t> splitCurveSegment(size_t nodeId1, size_t nodeId2,
                                                size_t newNodeId, double tMid);

private:
    MeshData2D& meshData2D_;
};

} // namespace Meshing
