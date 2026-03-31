#pragma once

#include "../Base/IElement.h"
#include "MeshData2D.h"
#include "Meshing/Data/CurveSegmentManager.h"
#include <memory>
#include <string>
#include <utility>

#include "Common/Types.h"

namespace Meshing
{

class MeshMutator2D
{
public:
    explicit MeshMutator2D(MeshData2D& meshData);

    // Node operations
    size_t addNode(const Point2D& coordinates);
    size_t addBoundaryNode(const Point2D& coordinates, const std::vector<std::string>& geometryIds);
    void moveNode(size_t id, const Point2D& newCoords);
    void removeNode(size_t id);

    // Element operations
    size_t addElement(std::unique_ptr<IElement> element);
    void removeElement(size_t id);

    // Curve segment operations
    void addCurveSegment(const CurveSegment& segment);
    void setCurveSegmentManager(CurveSegmentManager manager);
    std::pair<size_t, size_t> splitCurveSegment(size_t nodeId1, size_t nodeId2,
                                                 size_t newNodeId, double tMid);
    void clearCurveSegments();

private:
    MeshData2D& meshData_;
};

} // namespace Meshing
