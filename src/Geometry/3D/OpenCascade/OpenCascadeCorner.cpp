#include "OpenCascadeCorner.h"
#include <BRep_Tool.hxx>
#include <gp_Pnt.hxx>
#include <sstream>

namespace Geometry3D
{

OpenCascadeCorner::OpenCascadeCorner(const TopoDS_Vertex& vertex) :
    vertex_(vertex)
{
}

Meshing::Point3D OpenCascadeCorner::getPoint() const
{
    gp_Pnt point = BRep_Tool::Pnt(vertex_);
    return Meshing::Point3D(point.X(), point.Y(), point.Z());
}

std::string OpenCascadeCorner::getId() const
{
    std::ostringstream oss;
    oss << "OpenCascadeCorner_" << std::hex << reinterpret_cast<uintptr_t>(&vertex_);
    return oss.str();
}

} // namespace Geometry3D