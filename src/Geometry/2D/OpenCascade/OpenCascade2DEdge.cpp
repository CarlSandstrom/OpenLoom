#include "OpenCascade2DEdge.h"
#include <GCPnts_AbscissaPoint.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Precision.hxx>
#include <gp_Pnt2d.hxx>
#include <sstream>

namespace Geometry2D
{

OpenCascade2DEdge::OpenCascade2DEdge(const Handle(Geom2d_Curve) & curve,
                                     const std::string& id) :
    curve_(curve),
    id_(id),
    firstParameter_(curve->FirstParameter()),
    lastParameter_(curve->LastParameter())
{
    if (id_.empty())
    {
        std::ostringstream oss;
        oss << "OpenCascade2DEdge_" << std::hex
            << reinterpret_cast<uintptr_t>(curve.get());
        id_ = oss.str();
    }
}

Meshing::Point2D OpenCascade2DEdge::getPoint(double t) const
{
    gp_Pnt2d point = curve_->Value(t);
    return Meshing::Point2D(point.X(), point.Y());
}

Meshing::Point2D OpenCascade2DEdge::getStartPoint() const
{
    return getPoint(firstParameter_);
}

Meshing::Point2D OpenCascade2DEdge::getEndPoint() const
{
    return getPoint(lastParameter_);
}

std::pair<double, double> OpenCascade2DEdge::getParameterBounds() const
{
    return {firstParameter_, lastParameter_};
}

double OpenCascade2DEdge::getLength() const
{
    Geom2dAdaptor_Curve adaptor(curve_);

    // Use GCPnts_AbscissaPoint to compute the actual arc length
    double length = GCPnts_AbscissaPoint::Length(adaptor, firstParameter_, lastParameter_);

    if (length > Precision::Confusion())
    {
        return length;
    }

    // Fallback: return parameter range (not ideal but better than 0)
    return lastParameter_ - firstParameter_;
}

std::string OpenCascade2DEdge::getId() const
{
    return id_;
}

} // namespace Geometry2D
