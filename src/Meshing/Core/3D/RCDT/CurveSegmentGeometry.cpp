#include "Meshing/Core/3D/RCDT/CurveSegmentGeometry.h"

#include "Common/Exceptions/GeometryException.h"
#include "Geometry/3D/Base/GeometryCollection3D.h"
#include "Geometry/3D/Base/IEdge3D.h"

namespace Meshing
{

Point3D CurveSegmentGeometry::splitPoint(const CurveSegment& segment,
                                         const Geometry3D::GeometryCollection3D& geometry)
{
    const Geometry3D::IEdge3D* edge = geometry.getEdge(segment.edgeId);
    OPENLOOM_REQUIRE_NOT_NULL(edge, segment.edgeId);

    const double tMid = edge->getParameterAtArcLengthFraction(segment.tStart, segment.tEnd, 0.5);
    return edge->getPoint(tMid);
}

} // namespace Meshing
