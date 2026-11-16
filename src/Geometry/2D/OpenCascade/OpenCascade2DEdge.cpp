#include "OpenCascade2DEdge.h"
#include <GCPnts_AbscissaPoint.hxx>
#include <Geom2dAdaptor_Curve.hxx>
#include <Precision.hxx>
#include <cstdint>
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

Meshing::Point2D OpenCascade2DEdge::getPoint(double parameter) const
{
    auto point = curve_->Value(parameter);
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

    auto length = GCPnts_AbscissaPoint::Length(adaptor, firstParameter_, lastParameter_);

    if (length > Precision::Confusion())
    {
        return length;
    }

    // A degenerate curve has no arc length; fall back to its parameter range
    return lastParameter_ - firstParameter_;
}

std::string OpenCascade2DEdge::getId() const
{
    return id_;
}

} // namespace Geometry2D
