#pragma once

#include "Common/Types.h"
#include "Meshing/Data/2D/MeshData2D.h"
#include "Meshing/Data/Base/IElement.h"
#include "Meshing/Data/CurveSegmentManager.h"

#include <memory>
#include <utility>

namespace Meshing
{

class MeshMutator2D
{
public:
    explicit MeshMutator2D(MeshData2D& meshData2D);

    NodeId addNode(const Point2D& coordinates);

    ElementId addElement(std::unique_ptr<IElement> element);
    void removeElement(ElementId id);

    void setCurveSegmentManager(CurveSegmentManager curveSegmentManager);
    std::pair<SegmentId, SegmentId> splitCurveSegment(NodeId nodeId1, NodeId nodeId2,
                                                      NodeId newNodeId, double tMid);

private:
    MeshData2D& meshData2D_;
};

} // namespace Meshing
