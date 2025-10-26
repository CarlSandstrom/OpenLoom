#include "OpenCascadeCorner.h"
#include <BRep_Tool.hxx>
#include <gp_Pnt.hxx>
#include <sstream>

namespace Geometry
{

OpenCascadeCorner::OpenCascadeCorner(const TopoDS_Vertex& vertex) :
    vertex_(vertex)
{
}

std::array<double, 3> OpenCascadeCorner::getPoint() const
{
    gp_Pnt point = BRep_Tool::Pnt(vertex_);
    return {point.X(), point.Y(), point.Z()};
}

std::string OpenCascadeCorner::getId() const
{
    std::ostringstream oss;
    oss << "OpenCascadeCorner_" << std::hex << reinterpret_cast<uintptr_t>(&vertex_);
    return oss.str();
}

} // namespace Geometry