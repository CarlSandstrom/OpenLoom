#include "GeometryOperations2D.h"
#include "IFace2D.h"

namespace Geometry2D
{

GeometryOperations2D::GeometryOperations2D(const GeometryCollection2D& geometry) :
    geometry_(geometry)
{
}

bool GeometryOperations2D::isPointInsideDomain(const Meshing::Point2D& point, const IFace2D& domainFace)
{
    PointLocation location = domainFace.classify(point);
    return location == PointLocation::Inside ||
           location == PointLocation::OnBoundary;
}

} // namespace Geometry2D
