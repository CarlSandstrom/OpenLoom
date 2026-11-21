#include "Meshing/Core/3D/General/RobustPredicates3D.h"
#include "Meshing/Core/3D/General/OrientationDeterminant3D.h"

namespace Meshing
{

namespace
{

int halfOpenPlaneSide(const Point3D& vertex0, const Point3D& vertex1, const Point3D& vertex2, const Point3D& point);

} // namespace

int RobustPredicates3D::orientationSign(const Point3D& point0,
                                        const Point3D& point1,
                                        const Point3D& point2,
                                        const Point3D& point3)
{
    return OrientationDeterminant3D::sign(point0, point1, point2, point3);
}

bool RobustPredicates3D::segmentCrossesTriangle(const Point3D& segmentStart,
                                                const Point3D& segmentEnd,
                                                const Point3D& vertex0,
                                                const Point3D& vertex1,
                                                const Point3D& vertex2)
{
    // The segment's ends must be on opposite sides of the triangle's plane,
    // an end lying in it counting as on its positive side. Equal sides also
    // reject the coplanar case.
    int startSide = halfOpenPlaneSide(vertex0, vertex1, vertex2, segmentStart);
    int endSide = halfOpenPlaneSide(vertex0, vertex1, vertex2, segmentEnd);
    if (startSide == endSide)
        return false;

    // The plane crossing point (never computed explicitly) is inside the
    // triangle iff the segment passes the same side of all 3 edges, taken in
    // a consistent winding order around the triangle -- i.e. these three
    // orientation tests all agree in sign. Any of them landing on exactly 0
    // means the crossing touches an edge or vertex rather than the
    // triangle's interior, so it's rejected rather than treated as a match.
    int edge01 = orientationSign(segmentStart, segmentEnd, vertex0, vertex1);
    int edge12 = orientationSign(segmentStart, segmentEnd, vertex1, vertex2);
    int edge20 = orientationSign(segmentStart, segmentEnd, vertex2, vertex0);
    return edge01 != 0 && edge01 == edge12 && edge12 == edge20;
}

namespace
{

// The side of the triangle's plane the point lies on, a point exactly in the plane counting as
// on the positive side. A dual edge can end at an orthocenter that lies on a planar face to
// within rounding -- the exact one may be closer than one ulp -- and every face around it then
// has a dual edge ending there. Assigning the end one side decides all of them consistently, so
// exactly the faces whose other end lies opposite are crossed (OPE-214).
int halfOpenPlaneSide(const Point3D& vertex0, const Point3D& vertex1, const Point3D& vertex2, const Point3D& point)
{
    return RobustPredicates3D::orientationSign(vertex0, vertex1, vertex2, point) < 0 ? -1 : 1;
}

} // namespace

} // namespace Meshing
