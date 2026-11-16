#include "Meshing/Core/2D/General/ConstraintChecker2D.h"

#include "Meshing/Core/2D/General/GeometryUtilities2D.h"
#include "Meshing/Core/2D/Periodic/PeriodicMeshData2D.h"

namespace Meshing
{

ConstraintChecker2D::ConstraintChecker2D(const MeshData2D& mesh) :
    mesh_(mesh)
{
}

ConstraintChecker2D::ConstraintChecker2D(const MeshData2D& mesh, PeriodicMeshData2D* periodicData) :
    mesh_(mesh),
    periodicData_(periodicData)
{
}

bool ConstraintChecker2D::isSegmentEncroached(const CurveSegment& segment, const Point2D& point) const
{
    auto node1 = mesh_.getNode(segment.nodeId1);
    auto node2 = mesh_.getNode(segment.nodeId2);
    auto circle = GeometryUtilities2D::createDiametralCircle(node1->getCoordinates(), node2->getCoordinates());

    // In a periodic domain, the candidate point may be in a different period copy
    // than the segment. Shift it to the nearest copy relative to the segment midpoint
    // so the diametral circle test uses the correct distance.
    auto testPoint = periodicData_ ? periodicData_->nearestCopy(point, circle.center) : point;

    return GeometryUtilities2D::isPointInsideCircle(circle, testPoint);
}

} // namespace Meshing
