#include "ConstraintChecker2D.h"
#include "GeometryUtilities2D.h"

namespace Meshing
{

ConstraintChecker2D::ConstraintChecker2D(const MeshData2D& mesh) :
    mesh_(mesh)
{
}

bool ConstraintChecker2D::isSegmentEncroached(const ConstrainedSegment2D& segment, const Point2D& point) const
{
    const auto* node1 = mesh_.getNode(segment.nodeId1);
    const auto* node2 = mesh_.getNode(segment.nodeId2);
    DiametralCircle2D circle = GeometryUtilities2D::createDiametralCircle(node1->getCoordinates(), node2->getCoordinates());
    return GeometryUtilities2D::isPointInDiametralCircle(circle, point);
}

} // namespace Meshing
